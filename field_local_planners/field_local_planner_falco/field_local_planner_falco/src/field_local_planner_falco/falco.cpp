#include <math.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <field_local_planner_falco/falco.hpp>

namespace field_local_planner {

Falco::Falco() : BaseLocalPlanner() {}

void Falco::loadParameters(const Falco::Parameters& p) {
  parameters_ = p;

  loadPathsFromFile(parameters_.config_folder);

  // Precompute robot diameters
  robot_diameter_ = std::sqrt(std::pow(parameters_.robot_length, 2) + std::pow(parameters_.robot_width, 2));
}

Twist Falco::computeTwist() {
  // check collisions and compute free path
  computeFreePath();

  // Select carrot from path
  computeIntermediateCarrot();

  // Compute command from carrot
  double error_heading_to_carrot = atan2(intermediate_carrot_(1), intermediate_carrot_(0));
  double error_distance_to_carrot = hypot(intermediate_carrot_(1), intermediate_carrot_(0));
  double velocity_direction = 1.0;

  // if(parameters_.goal_behind_mode_ == 1 ) {
  //   if (intermediate_carrot_(0) <=0 && (dT_base_goal_.translation().x() <0)){
  //     error_heading_to_carrot = utils::mod2pi(error_heading_to_carrot + M_PI);
  //     velocity_direction = -1.0;
  //   // } else if((dT_base_goal_.translation().x() <0)) {
  //   //   error_heading_to_carrot = utils::mod2pi(error_heading_to_carrot + M_PI);
  //   //   velocity_direction = -1.0;
  //   }
  // }

  Twist twist;
  twist(0) = 0.0;  // Angular x
  twist(1) = 0.0;  // Angular y
  if (error_distance_to_carrot >= parameters_.look_ahead_distance)
    twist(2) = error_heading_to_carrot * parameters_.angular_gain_p;
  else
    twist(2) = heading_towards_goal_ * parameters_.angular_gain_p;

  // Linear
  twist(3) = velocity_direction * parameters_.max_linear_velocity_x * distance_to_goal_;
  twist(4) = 0.0;  // Linear y
  twist(5) = 0.0;  // Linear z

  return twist;
}

Path Falco::computePath() {
  // TODO
}

//-------------------------------------------------------------------------------------------------
// Check precomputed paths collisions
//-------------------------------------------------------------------------------------------------
void Falco::computeFreePath() {
  // Path range
  path_range_ = parameters_.sensor_range;
  path_range_ = std::max(path_range_, parameters_.min_path_range);

  // Path scale
  // Store the path scale temporarily
  path_scale_ = parameters_.path_scale;
  path_scale_ = std::max(path_scale_, parameters_.min_path_scale);

  // Main iteration to check the paths
  while (path_scale_ >= parameters_.min_path_scale && path_range_ >= parameters_.min_path_range) {
    // std::cout << "path_scale: " << path_scale_ << ", path_range_: " << path_range_);
    clearPathsLists();
    computeMapCollisions();
    computePathScores();

    feasible_path_found_ = findBestPath();
    // std::cout << "[Falco] found best path: " << feasible_path_found_);
    if (feasible_path_found_) {
      break;
    }
  }
}

void Falco::clearPathsLists() {
  // ROS_INFO("[Falco] clear path lists");
  // Clear path list
  for (int i = 0; i < 36 * NUM_PATHS; i++) {
    clear_path_list_[i] = 0;
    // path_penalty_list_[i] = 0.0;
    path_deformation_list_[i] = 0.0;
    path_traversability_score_[i] = 0.0;
    path_traversability_crossed_voxels_[i] = 0;
  }

  // clear score of group
  for (int i = 0; i < 36 * NUM_GROUPS; i++) {
    path_group_likelihood_[i] = 0.0;
  }
}

void Falco::computeMapCollisions() {
  // Iterate over all the cloud points to find collisions
  int cloud_size = point_cloud_->points.size();
  for (int i = 0; i < point_cloud_->size(); i++) {
    // Precompute all the properties about the voxel that are reused later
    const float x = (point_cloud_->points[i].x - parameters_.robot_clearance) / path_scale_;  // normalized by path_scale
    const float y = (point_cloud_->points[i].y - parameters_.robot_clearance) / path_scale_;  // normalized by path_scale

    // Get traversability
    // Defined in the intensity channel
    const float traversability = point_cloud_->points[i].intensity;

    // Check the score
    bool is_traversable = traversability > parameters_.traversable_thr;

    // Compute distance and angle to obstacle
    float distance_to_obstacle = std::sqrt(x * x + y * y);
    float angle_to_obstacle = std::atan2(y, x);

    // If the obstacle is within the path ranges, check collisions
    if (distance_to_obstacle < path_range_ / path_scale_ &&
        (distance_to_obstacle <= ((distance_to_goal_ + parameters_.goal_clearance) / path_scale_) || !parameters_.use_path_crop_by_goal)) {
      // Iterate over all the group paths to check if the obstacle is in their way
      for (int rot_dir = 0; rot_dir < 36; rot_dir++) {
        // get angle between [-pi, pi]
        float rot_ang = utils::deg2rad(10.0 * rot_dir - 180.0);

        // Rotate the point to express it in the path group frame
        float x2 = cos(rot_ang) * x + sin(rot_ang) * y;
        float y2 = -sin(rot_ang) * x + cos(rot_ang) * y;

        // Get indices in path collision grid (correspondences)
        float scale_y =
            (x2 / GRID_VOXEL_OFFSET_X) + (SEARCH_RADIUS / GRID_VOXEL_OFFSET_Y) * (GRID_VOXEL_OFFSET_X - x2) / GRID_VOXEL_OFFSET_X;

        int idx_x = int((GRID_VOXEL_OFFSET_X + GRID_VOXEL_SIZE / 2 - x2) / GRID_VOXEL_SIZE);
        int idx_y = int((GRID_VOXEL_OFFSET_Y + GRID_VOXEL_SIZE / 2 - y2 / scale_y) / GRID_VOXEL_SIZE);

        // If the indices fall into the correspondences grid
        if (idx_x >= 0 && idx_x < NUM_GRID_VOXELS_X && idx_y >= 0 && idx_y < NUM_GRID_VOXELS_Y) {
          // Compute index to find correspondences
          int idx = NUM_GRID_VOXELS_Y * idx_x + idx_y;

          // Iterate all the paths that pass through the obstacle voxel
          for (int j = 0; j < correspondences_[idx].size(); j++) {
            // Check traversability
            if (!is_traversable) {
              // Update the number of actual collisions with the path
              clear_path_list_[NUM_PATHS * rot_dir + correspondences_[idx][j]]++;
            }
            // Update path score
            path_traversability_score_[NUM_PATHS * rot_dir + correspondences_[idx][j]] += traversability;
            path_traversability_crossed_voxels_[NUM_PATHS * rot_dir + correspondences_[idx][j]]++;
            // Original previous criterion used the largest height to compute a "penalty"
            // TODO: check the previous approach works
          }
        }
      }
    }

    // Check if the point collides with the robot diameter
    if (parameters_.check_rotational_collisions && !is_traversable) {
      if ((distance_to_obstacle < robot_diameter_ / path_scale_) &&
          ((fabs(x) < parameters_.robot_length / path_scale_ / 2.0) || (fabs(y) < parameters_.robot_width / path_scale_ / 2.0))) {
        // TODO
      }
    }  // Check rotational collisions
  }    // Map points iteration
}

void Falco::computePathScores() {
  // Compute the scores for each path
  for (int i = 0; i < 36 * NUM_PATHS; i++) {
    int rot_dir = int(i / NUM_PATHS);
    float rot_ang = utils::deg2rad(10.0 * rot_dir - 180.0);
    float ang_diff = utils::mod2pi(heading_towards_goal_ - rot_ang);

    // select the paths that have less than a fixed number of collisions detected
    if (clear_path_list_[i] <= parameters_.point_per_path_thr) {
      // Compute the main scores/probabilities to select the path

      // Goal probability
      // A Von mises distribution centered in the direction of the goal
      float goal_probability = utils::vonMises(end_dir_path_list_[i % NUM_PATHS] + rot_ang, heading_towards_goal_, parameters_.goal_cost);

      // Traversability probability
      // Average probability over the traversability of all the voxels crossed by the path
      float traversability_probability =
          exp(parameters_.traversability_cost * path_traversability_score_[i] / path_traversability_crossed_voxels_[i]);

      // Differential drive probability
      // If the vehicle is a differential drive, the most probable paths should be in front or the back of the robot
      // Modeled as a mixture of Von Mises
      float differential_drive_probability = 1.0;
      // if(parameters_.differential_drive){
      //   differential_drive_probability = 0.5*(utils::vonMises(rot_ang, 0, parameters_.differential_mode_cost) + utils::vonMises(rot_ang,
      //   M_PI, parameters_.differential_mode_cost));
      // }

      // Compute score as the likelihood of all the previous factors
      float score_likelihood = goal_probability * traversability_probability * differential_drive_probability;

      // Compute group index
      int group = NUM_GROUPS * rot_dir + path_list_[i % NUM_PATHS];

      // Store score per path
      path_likelihood_[i] = score_likelihood;

      // Add the score of the path to the score of the group
      if (score_likelihood > 0) {
        path_group_likelihood_[NUM_GROUPS * rot_dir + path_list_[i % NUM_PATHS]] += score_likelihood;
      }
    }
  }
}

bool Falco::findBestPath() {
  // ROS_INFO("[Falco] find best path");
  // Select the group with the maximum score
  // This is a simple linear search
  best_path_group_score_ = 0;
  float min_score = std::numeric_limits<float>::infinity();
  best_path_group_id_ = -1;
  for (int i = 0; i < 36 * NUM_GROUPS; i++) {
    // int rot_dir = int(i / NUM_GROUPS);
    // float rot_ang = utils::deg2rad(10.0 * rot_dir - 180.0);

    // select path with maximum score that doesn't collide with obstacles
    if (best_path_group_score_ < path_group_likelihood_[i]) {
      best_path_group_score_ = path_group_likelihood_[i];
      best_path_group_id_ = i;
    }
    if (min_score > path_group_likelihood_[i]) {
      min_score = path_group_likelihood_[i];
    }
  }

  // Prepare path structure
  best_path_.clear();
  if (best_path_group_id_ >= 0) {
    int rot_dir = int(best_path_group_id_ / NUM_GROUPS);
    float rot_ang = utils::deg2rad(10.0 * rot_dir - 180.0);

    best_path_group_id_ = best_path_group_id_ % NUM_GROUPS;
    int selected_path_length = precomputed_paths_[best_path_group_id_]->points.size();
    best_path_.resize(selected_path_length);

    for (int i = 0; i < selected_path_length; i++) {
      float x = precomputed_paths_[best_path_group_id_]->points[i].x;
      float y = precomputed_paths_[best_path_group_id_]->points[i].y;
      float z = precomputed_paths_[best_path_group_id_]->points[i].z;
      float dis = sqrt(x * x + y * y);

      if (dis <= path_range_ / path_scale_ && dis <= distance_to_goal_ / path_scale_) {
        best_path_[i] = Vector3(path_scale_ * (cos(rot_ang) * x - sin(rot_ang) * y), path_scale_ * (sin(rot_ang) * x + cos(rot_ang) * y),
                                path_scale_ * z);
      } else {
        best_path_.resize(i);
        break;
      }
    }

    // Fill free paths (for visualization)
    free_paths_->clear();
    for (int i = 0; i < 36 * NUM_PATHS; i++) {
      int rot_dir = int(i / NUM_PATHS);
      // float rot_ang = (10.0 * rot_dir - 180.0) * M_PI / 180;
      float rot_ang = utils::deg2rad(10.0 * rot_dir - 180.0);

      pcl::PointXYZI point;
      if (clear_path_list_[i] < parameters_.point_per_path_thr) {
        int free_path_length = paths_[i % NUM_PATHS]->points.size();

        for (int j = 0; j < free_path_length; j++) {
          point = paths_[i % NUM_PATHS]->points[j];

          float path_deformation = path_deformation_list_[i];

          float x = point.x;
          float y = point.y;
          float z = point.z;

          point.x = path_scale_ * (cos(rot_ang) * x - sin(rot_ang) * y);
          point.y = path_scale_ * (sin(rot_ang) * x + cos(rot_ang) * y);
          point.z = path_scale_ * z + path_deformation;
          // point.z = path_group_likelihood_[NUM_GROUPS * rot_dir + path_list_[i % NUM_PATHS]];
          point.intensity =
              path_group_likelihood_[NUM_GROUPS * rot_dir +
                                     path_list_[i % NUM_PATHS]];  // path_likelihood_[i]; //clear_path_list_[i]; //path_likelihood_[i];
          // point.intensity = path_likelihood_[i];

          float dis = sqrt(x * x + y * y);
          if (dis <= path_range_ / path_scale_ &&
              (dis <= (distance_to_goal_ + parameters_.goal_clearance) / path_scale_ || !parameters_.use_path_crop_by_goal)) {
            free_paths_->push_back(point);
          }
        }
      }
    }
  }

  if (best_path_group_id_ < 0) {
    if (path_scale_ >= parameters_.min_path_scale + parameters_.path_scale_step) {
      path_scale_ -= parameters_.path_scale_step;
      path_range_ = parameters_.sensor_range * path_scale_ / parameters_.path_scale;
    } else {
      path_range_ -= parameters_.path_range_step;
    }
  } else {
    return true;
  }

  return false;
}

//-------------------------------------------------------------------------------------------------
// Compute intermediate carrot
//-------------------------------------------------------------------------------------------------
void Falco::computeIntermediateCarrot() {
  if (best_path_.empty()) {
    std::cout << "Best path is empty" << std::endl;
    return;
  }

  // Search carrot in path that is farther than look_ahead_distance
  intermediate_carrot_ = Vector3::Zero();
  for (auto point : best_path_) {
    double distance = point.norm();
    if (distance > parameters_.look_ahead_distance) {
      intermediate_carrot_ = point;
      break;
    }
  }

  // Get end of path
  path_end_ = best_path_.back();
}

//-------------------------------------------------------------------------------------------------
// Load files
//-------------------------------------------------------------------------------------------------
void Falco::loadPathsFromFile(std::string folder) {
  // Initialize required variables
  free_paths_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

  for (int i = 0; i < NUM_MAP_CLOUD_STACK; i++) {
    map_cloud_stack[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }
  for (int i = 0; i < NUM_GROUPS; i++) {
    precomputed_paths_[i].reset(new pcl::PointCloud<pcl::PointXYZ>());
  }
  for (int i = 0; i < NUM_PATHS; i++) {
    paths_[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }
  for (int i = 0; i < NUM_GRID_VOXELS; i++) {
    correspondences_[i].resize(0);
  }

  std::cout << "Loading files from folder " << folder << "" << std::endl;

  // Load the files
  loadPrecomputedPaths(folder);
  loadPaths(folder);
  loadPathsList(folder);
  loadCorrespondences(folder);
}

FILE* Falco::openFile(std::string& file) {
  std::cout << "Opening file " << file << std::endl;

  FILE* file_ptr = fopen(file.c_str(), "r");
  if (file_ptr == NULL) {
    std::cout << "Error: Cannot read input files, exit." << std::endl;
    exit(1);
  }

  return file_ptr;
}

int Falco::readPlyHeader(FILE* file_ptr) {
  char str[50];
  int val, point_num;

  std::string str_cur, str_last;
  while (str_cur != "end_header") {
    val = fscanf(file_ptr, "%s", str);
    if (val != 1) {
      std::cout << "Error reading input files, exit." << std::endl;
      exit(1);
    }

    str_last = str_cur;
    str_cur = std::string(str);

    if (str_cur == "vertex" && str_last == "element") {
      val = fscanf(file_ptr, "%d", &point_num);
      if (val != 1) {
        std::cout << "Error reading input files, exit." << std::endl;
        exit(1);
      }
    }
  }

  return point_num;
}

void Falco::loadPrecomputedPaths(std::string folder) {
  std::string file_name = folder + "precomputed_paths.ply";

  FILE* file_ptr = openFile(file_name);
  int point_num = readPlyHeader(file_ptr);

  pcl::PointXYZ point;
  int val1, val2, val3, val4, group_id;
  for (int i = 0; i < point_num; i++) {
    val1 = fscanf(file_ptr, "%f", &point.x);
    val2 = fscanf(file_ptr, "%f", &point.y);
    val3 = fscanf(file_ptr, "%f", &point.z);
    val4 = fscanf(file_ptr, "%d", &group_id);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1) {
      std::cout << "Error reading input files, exit." << std::endl;
      exit(1);
    }

    if (group_id >= 0 && group_id < NUM_GROUPS) {
      precomputed_paths_[group_id]->push_back(point);
    }
  }

  fclose(file_ptr);
}

void Falco::loadPathsList(std::string folder) {
  std::string file_name = folder + "path_list.ply";

  FILE* file_ptr = openFile(file_name);
  if (NUM_PATHS != readPlyHeader(file_ptr)) {
    std::cout << "Error: Incorrect path number, exit." << std::endl;
    exit(1);
  }

  int val1, val2, val3, val4, val5, path_id, group_id;
  float end_x, end_y, end_z;
  for (int i = 0; i < NUM_PATHS; i++) {
    val1 = fscanf(file_ptr, "%f", &end_x);
    val2 = fscanf(file_ptr, "%f", &end_y);
    val3 = fscanf(file_ptr, "%f", &end_z);
    val4 = fscanf(file_ptr, "%d", &path_id);
    val5 = fscanf(file_ptr, "%d", &group_id);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
      std::cout << "Error reading input files, exit." << std::endl;
      exit(1);
    }

    if (path_id >= 0 && path_id < NUM_PATHS && group_id >= 0 && group_id < NUM_GROUPS) {
      path_list_[path_id] = group_id;
      end_dir_path_list_[path_id] = 2.0 * atan2(end_y, end_x);
    }
  }

  fclose(file_ptr);
}

void Falco::loadPaths(std::string folder) {
  std::string file_name = folder + "paths.ply";

  FILE* file_ptr = openFile(file_name);
  int point_num = readPlyHeader(file_ptr);

  pcl::PointXYZI point;
  int point_skip_num = 30;
  int point_skip_count = 0;
  int val1, val2, val3, val4, val5, path_id;
  for (int i = 0; i < point_num; i++) {
    val1 = fscanf(file_ptr, "%f", &point.x);
    val2 = fscanf(file_ptr, "%f", &point.y);
    val3 = fscanf(file_ptr, "%f", &point.z);
    val4 = fscanf(file_ptr, "%d", &path_id);
    val5 = fscanf(file_ptr, "%f", &point.intensity);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
      std::cout << "Error reading input files, exit." << std::endl;
      exit(1);
    }

    if (path_id >= 0 && path_id < NUM_PATHS) {
      point_skip_count++;
      if (point_skip_count > point_skip_num) {
        paths_[path_id]->push_back(point);
        point_skip_count = 0;
      }
    }
  }

  fclose(file_ptr);
}

void Falco::loadCorrespondences(std::string folder) {
  std::string file_name = folder + "correspondences.txt";

  FILE* file_ptr = fopen(file_name.c_str(), "r");
  if (file_ptr == NULL) {
    std::cout << "Error: Cannot read input files, exit." << std::endl;
    exit(1);
  }

  int val1, grid_voxel_id, path_id;
  for (int i = 0; i < NUM_GRID_VOXELS; i++) {
    val1 = fscanf(file_ptr, "%d", &grid_voxel_id);
    if (val1 != 1) {
      std::cout << "Error reading input files, exit." << std::endl;
      exit(1);
    }

    while (1) {
      val1 = fscanf(file_ptr, "%d", &path_id);
      if (val1 != 1) {
        std::cout << "Error reading input files, exit." << std::endl;
        exit(1);
      }

      if (path_id != -1) {
        if (grid_voxel_id >= 0 && grid_voxel_id < NUM_GRID_VOXELS && path_id >= 0 && path_id < NUM_PATHS) {
          correspondences_[grid_voxel_id].push_back(path_id);
        }
      } else {
        break;
      }
    }
  }

  fclose(file_ptr);
}

}  // namespace field_local_planner

// //-------------------------------------------------------------------------------------------------
// // Constructor and parameters
// //-------------------------------------------------------------------------------------------------
// FalcoController::FalcoController(const ControllerParameters& params) :
//   ControllerBase(params),
//   last_linear_velocity_(0.0) {

//   node_handle_ = ros::NodeHandle("~falco");

//   // Update parameters
//   ROS_WARN("[ControllerBase] Setting up Controller Parameters");
//   updateControllerParameters(node_handle_);

//   // Setup Dynamic reconfigure
//   ROS_WARN("[ControllerBase] Setting up Dynamic reconfigure");
//   setupDynamicReconfigureServer(node_handle_);

//   // Setup visualizations
//   ROS_WARN("[ControllerBase] Setting up Visualizations");
//   setupVisualizations(node_handle_);

//   collision_map_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZINormal>>();
//   collision_map_in_base_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZINormal>>();
//   collision_map_wnormals_in_base_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZINormal>>();

//   // load paths
//   loadPathsFromFile(parameters_.config_folder_);

//   // voxel filter
//   voxel_filter_.setLeafSize(parameters_.voxel_size_filter_, parameters_.voxel_size_filter_, parameters_.voxel_size_filter_);
// };

// void FalcoController::updateControllerParameters(ros::NodeHandle& node_handle) {
//   // Perceptive controller params
//   parameters_.config_folder_   = ros::package::getPath("field_local_planner") + "/config/controllers/falco/";
//   // falco params
//   parameters_.goal_clearance_              = utils::getParameterDefault<double>(node_handle, "goal_clearance", 0.1);
//   parameters_.forward_linear_gain_p_       = utils::getParameterDefault<double>(node_handle, "forward_linear_gain_p", 0.1);
//   parameters_.forward_angular_gain_p_      = utils::getParameterDefault<double>(node_handle, "forward_angular_gain_p", 0.1);
//   parameters_.linear_acceleration_         = utils::getParameterDefault<double>(node_handle, "linear_acceleration", 0.2);
//   parameters_.path_scale_                  = utils::getParameterDefault<double>(node_handle, "path_scale", 1.25);
//   parameters_.path_scale_step_             = utils::getParameterDefault<double>(node_handle, "path_scale_step", 0.25);
//   parameters_.min_path_scale_              = utils::getParameterDefault<double>(node_handle, "min_path_scale", 0.75);
//   parameters_.path_range_step_             = utils::getParameterDefault<double>(node_handle, "path_range_step", 0.5);
//   parameters_.min_path_range_              = utils::getParameterDefault<double>(node_handle, "min_path_range", 1.0);
//   parameters_.look_ahead_distance_         = utils::getParameterDefault<double>(node_handle, "look_ahead_distance", 0.5);
//   parameters_.differential_mode_cost_      = utils::getParameterDefault<double>(node_handle, "differential_mode_cost", 1.0);
//   parameters_.goal_cost_                   = utils::getParameterDefault<double>(node_handle, "goal_cost", 1.0);
//   parameters_.traversability_cost_         = utils::getParameterDefault<double>(node_handle, "traversability_cost", 1.0);
//   parameters_.point_per_path_thr_          = utils::getParameterDefault<int>(node_handle,  "point_per_path_thr", 2);
//   parameters_.check_rotational_collisions_ = utils::getParameterDefault<bool>(node_handle, "check_rotational_collisions", true);
//   parameters_.use_path_crop_by_goal_       = utils::getParameterDefault<bool>(node_handle, "crop_path_by_goal", true);

//   path_scale_ = parameters_.path_scale_;

//   // voxel filter
//   voxel_filter_.setLeafSize(parameters_.voxel_size_filter_, parameters_.voxel_size_filter_, parameters_.voxel_size_filter_);
// }

// void FalcoController::setupDynamicReconfigureServer(ros::NodeHandle& node_handle)
// {
//   // Create Dynamic Reconfigure server passing correct node handle
//   dynamic_reconfigure_server_ = std::make_shared<dynamic_reconfigure::Server<field_local_planner::FalcoControllerConfig>>(node_handle);

//   dynamic_reconfigure_callback_ = boost::bind(&FalcoController::dynamicReconfigureCallback, this, _1, _2);
//   dynamic_reconfigure_server_->setCallback(dynamic_reconfigure_callback_);
// }

// // void FalcoController::dynamicReconfigureCallback(field_local_planner::FalcoControllerConfig &config, uint32_t level)
// // {
// //   // Falco parameters
// //   utils::assignAndPrintDiff("goal_clearance", parameters_.goal_clearance_, config.goal_clearance);
// //   utils::assignAndPrintDiff("forward_linear_gain_p", parameters_.forward_linear_gain_p_, config.forward_linear_gain_p);
// //   utils::assignAndPrintDiff("forward_angular_gain_p", parameters_.forward_angular_gain_p_, config.forward_angular_gain_p);
// //   utils::assignAndPrintDiff("linear_acceleration", parameters_.linear_acceleration_, config.linear_acceleration);
// //   utils::assignAndPrintDiff("path_scale", parameters_.path_scale_, config.path_scale);
// //   utils::assignAndPrintDiff("path_scale_step", parameters_.path_scale_step_, config.path_scale_step);
// //   utils::assignAndPrintDiff("min_path_scale", parameters_.min_path_scale_, config.min_path_scale);
// //   utils::assignAndPrintDiff("path_range_step", parameters_.path_range_step_, config.path_range_step);
// //   utils::assignAndPrintDiff("min_path_range", parameters_.min_path_range_, config.min_path_range);
// //   utils::assignAndPrintDiff("look_ahead_distance", parameters_.look_ahead_distance_, config.look_ahead_distance);
// //   utils::assignAndPrintDiff("differential_mode_cost", parameters_.differential_mode_cost_, config.differential_mode_cost);
// //   utils::assignAndPrintDiff("goal_cost", parameters_.goal_cost_, config.goal_cost);
// //   utils::assignAndPrintDiff("traversability_cost", parameters_.traversability_cost_, config.traversability_cost);
// //   utils::assignAndPrintDiff("point_per_path_thr", parameters_.point_per_path_thr_, config.point_per_path_thr);
// //   utils::assignAndPrintDiff("check_rotational_collisions", parameters_.check_rotational_collisions_,
// config.check_rotational_collisions);
// //   utils::assignAndPrintDiff("use_path_crop_by_goal", parameters_.use_path_crop_by_goal_, config.use_path_crop_by_goal);

// //   path_scale_ = parameters_.path_scale_;
// // }

// // //-------------------------------------------------------------------------------------------------
// // // Controller methods
// // //-------------------------------------------------------------------------------------------------
// // void FalcoController::customPreProcessLocalMap() {
// //   if(new_elevation_map_){
// //     profiler_ptr_->startEvent("0.1.convert_map_to_cloud");
// //     // If we receive a new elevation map, we must convert it to base frame
// //     convertElevationMapToCloud();
// //     profiler_ptr_->endEvent("0.1.convert_map_to_cloud");
// //   }

// //   // Otherwise, we must update the map accordingly with the new pose updates
// //   profiler_ptr_->startEvent("0.2.update_elevation_map_frame");
// //   updateElevationMapFrame();
// //   profiler_ptr_->endEvent("0.2.update_elevation_map_frame");

// // }

// // void FalcoController::customPreProcessController() {

// //   // Check path collisions
// //   // ROS_INFO("Computing free path");
// //   profiler_ptr_->startEvent("1.3.compute_free_path");
// //   computeFreePath();
// //   profiler_ptr_->endEvent("1.3.compute_free_path");

// //   // Compute velocity command
// //   // ROS_INFO("Computing intermediate carrot");
// //   profiler_ptr_->startEvent("1.4.compute_carrot");
// //   computeIntermediateCarrot();
// //   profiler_ptr_->endEvent("1.4.compute_carrot");
// // }

// // ControllerBase::OutputAction FalcoController::computeCommandGoalBehind(Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d&
// output_angular_velocity){

// //   // Compute angular velocity command
// //   output_linear_velocity(0) = 0.0;
// //   output_linear_velocity(1) = 0.0;
// //   output_linear_velocity(2) = 0.0;

// //   // compute angular velocity only command
// //   output_angular_velocity(0) = 0.0;
// //   output_angular_velocity(1) = 0.0;
// //   output_angular_velocity(2) = 0.0;

// //   last_linear_velocity_ = 0.0;

// //   return ControllerBase::OutputAction::SEND_COMMAND;

// //   // return computeCommandForward(output_linear_velocity, output_angular_velocity);
// // }

// // ControllerBase::OutputAction FalcoController::computeCommandTurnToGoal(Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d&
// output_angular_velocity){

// //   // TODO define a flag "initial_turn_to_goal" to allow the robot to rotate only the first time and use move forward aftwerwards

// //   double velocity_direction = 1.0;
// //   // // double error_heading_to_carrot = atan2(intermediate_carrot_(1), intermediate_carrot_(0));

// //   if ((dT_base_goal_.translation().x() <0) && (parameters_.goal_behind_mode_ == 1 )){
// //     velocity_direction = -1.0;
// //   }

// //   if (parameters_.yaw_exclusive_turns_) {
// //     velocity_direction = 0.0;
// //   }
// //   // output_linear_velocity(0)  = velocity_direction * cos(error_orientation_to_starting_pose_)*
// parameters_.max_forward_linear_velocity_;
// //   // output_linear_velocity(1)  = velocity_direction * sin(error_orientation_to_starting_pose_)*
// parameters_.max_forward_linear_velocity_;
// //   // output_linear_velocity(0)  = velocity_direction * cos(error_heading_to_carrot)* parameters_.max_forward_linear_velocity_;
// //   // output_linear_velocity(1)  = velocity_direction * sin(error_heading_to_carrot)* parameters_.max_forward_linear_velocity_;

// //   // compute angular velocity only command
// //   output_angular_velocity(0) = 0.0;
// //   output_angular_velocity(1) = 0.0;
// //   output_angular_velocity(2) = error_heading_to_goal_ * parameters_.angular_gain_p_;
// //   // output_angular_velocity(2) = error_heading_to_carrot * parameters_.angular_gain_p_;

// //   last_linear_velocity_ = 0.0;

// //   // return ControllerBase::OutputAction::SEND_COMMAND;

// //   return computeCommandForward(output_linear_velocity, output_angular_velocity);
// // }

// // ControllerBase::OutputAction FalcoController::computeCommandForward(Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d&
// output_angular_velocity){

// //   double error_heading_to_carrot = atan2(intermediate_carrot_(1), intermediate_carrot_(0));
// //   double error_distance_to_carrot = hypot(intermediate_carrot_(1), intermediate_carrot_(0));
// //   double velocity_direction = 1.0;

// //   ROS_INFO_STREAM_THROTTLE(1, "x to carrot: " << intermediate_carrot_(0) << ", y to carrot: " << intermediate_carrot_(1)<< ", distance
// to carrot: " << error_distance_to_carrot);

// //   if(parameters_.goal_behind_mode_ == 1 ) {
// //     if (intermediate_carrot_(0) <=0 && (dT_base_goal_.translation().x() <0)){
// //       error_heading_to_carrot = utils::mod2pi(error_heading_to_carrot + M_PI);
// //       velocity_direction = -1.0;
// //     // } else if((dT_base_goal_.translation().x() <0)) {
// //     //   error_heading_to_carrot = utils::mod2pi(error_heading_to_carrot + M_PI);
// //     //   velocity_direction = -1.0;
// //     }
// //   }

// //   // // Update velocity
// //   // if(error_distance_to_carrot >= parameters_.look_ahead_distance_){
// //   //   last_linear_velocity_+=parameters_.linear_acceleration_;
// //   // } else{
// //   //   last_linear_velocity_-=parameters_.linear_acceleration_;
// //   // }

// //   // // saturate by max velocity
// //   // last_linear_velocity_ = std::min(last_linear_velocity_, parameters_.max_forward_linear_velocity_);

// //   // output_linear_velocity(0)  = velocity_direction * parameters_.max_forward_linear_velocity_;
// //   output_linear_velocity(0)  = velocity_direction * parameters_.max_forward_linear_velocity_ * error_distance_to_goal_;
// //   // output_linear_velocity(0)  = velocity_direction * last_linear_velocity_;

// //   if(error_distance_to_carrot >= parameters_.look_ahead_distance_)
// //     output_angular_velocity(2) = error_heading_to_carrot * parameters_.forward_angular_gain_p_;
// //   else
// //     output_angular_velocity(2) = error_heading_to_goal_ * parameters_.forward_angular_gain_p_;

// //   return ControllerBase::OutputAction::SEND_COMMAND;
// // }

// // ControllerBase::OutputAction FalcoController::computeCommandTurnToDestination(Eigen::Vector3d& output_linear_velocity,
// Eigen::Vector3d& output_angular_velocity){
// //   double velocity_direction = 1.0;

// //   if ((dT_base_goal_.translation().x() <0) && (parameters_.goal_behind_mode_ == 1 )){
// //     velocity_direction = -1.0;
// //   }

// //   if (parameters_.yaw_exclusive_turns_) {
// //     velocity_direction = 0.0;
// //   }

// //   output_linear_velocity(0)  = velocity_direction * cos(error_heading_to_goal_)* parameters_.max_forward_linear_velocity_;
// //   output_linear_velocity(1)  = velocity_direction * sin(error_heading_to_goal_)* parameters_.max_forward_linear_velocity_;

// //   output_angular_velocity(2) = -error_orientation_to_goal_ * parameters_.angular_gain_p_;

// //   last_linear_velocity_ = 0.0;

// //   return ControllerBase::OutputAction::SEND_COMMAND;
// //   // return computeCommandForward(output_linear_velocity, output_angular_velocity);
// // }

// // ControllerBase::OutputAction FalcoController::computeCommandFinished(Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d&
// output_angular_velocity){
// //   ControllerBase::OutputAction action = ControllerBase::OutputAction::SEND_COMMAND;

// //   if (goals_.size() >0) {
// //     // ROS_INFO("Continuing to walk without stopping");
// //     action = SEND_NOTHING;
// //   }

// //   return action;
// // }

// // void FalcoController::computeCommandPostProcess(ControllerBase::OutputAction& action,
// //                                           Eigen::Vector3d& output_linear_velocity,
// //                                           Eigen::Vector3d& output_angular_velocity,
// //                                           std::vector<Eigen::Vector3d>& path_to_goal) {
// //   path_to_goal = best_path_;

// // }

// // //-------------------------------------------------------------------------------------------------
// // // Elevation map utils
// // //-------------------------------------------------------------------------------------------------
// // void FalcoController::convertElevationMapToCloud() {

// //   // Crop elevation map to sensor range (for path collision computation)
// //   grid_map::GridMap cropped_grid_map;
// //   grid_map::Position position( grid_map_.getPosition().x(),
// //                                grid_map_.getPosition().y());
// //   grid_map::Length length(2.0*parameters_.sensor_range_, 2.0*parameters_.sensor_range_);

// //   bool success;
// //   cropped_grid_map = grid_map_.getSubmap(position, length, success);

// //   // Convert to point_cloud and filter using voxel_filter
// //   collision_map_->clear();
// //   pcl::PointCloud<pcl::PointXYZINormal>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZINormal> ());

// //   sensor_msgs::PointCloud2 point_cloud;
// //   grid_map::GridMapRosConverter::toPointCloud(cropped_grid_map,
// //                                               {"normal_x",
// //                                                "normal_y",
// //                                                "normal_z",
// //                                                "elevation",
// //                                                "traversability",
// //                                                "slope"},
// //                                               "elevation", point_cloud);
// //   pcl::fromROSMsg(point_cloud, *tmp_cloud);
// //   voxel_filter_.setInputCloud(tmp_cloud);
// //   voxel_filter_.filter(*collision_map_);

// //   // Reset new elevation map flag
// //   new_elevation_map_ = false;

// //   // std::cout << "Terrain map has " << collision_map_->points.size() << " points");
// // }

// // void FalcoController::updateElevationMapFrame() {
// //   // std::lock_guard<std::mutex> lock(mutex_);

// //   if(grid_map_.getFrameId().size() == 0)
// //   {
// //     // no elevation map set
// //     return;
// //   }

// //   Eigen::Isometry3d map_to_base = Eigen::Isometry3d::Identity();
// //   tf::StampedTransform map_to_base_transform;
// //   tf_listener_.waitForTransform(parameters_.base_frame_, grid_map_.getFrameId(), ros::Time(0), ros::Duration(2.0));
// //   try {
// //     tf_listener_.lookupTransform(parameters_.base_frame_, grid_map_.getFrameId(), ros::Time(0), map_to_base_transform);

// //     tf::transformTFToEigen (map_to_base_transform, map_to_base);
// //   }
// //   catch (tf::TransformException& ex){
// //     std::cout << "%s",ex.what());
// //   }

// //   // ROS_INFO("Transforming frame");
// //   pcl::transformPointCloudWithNormals(*collision_map_, *collision_map_in_base_, map_to_base.matrix());

// //   // Compute traversability and as as intensity channel
// //   for(size_t i = 0; i<collision_map_in_base_->points.size(); i++) {
// //     collision_map_in_base_->points[i].intensity = computeTraversability(collision_map_in_base_->points[i]);
// //   }
// // }

// // //-------------------------------------------------------------------------------------------------
// // // Traversability computation
// // //-------------------------------------------------------------------------------------------------
// // float FalcoController::computeTraversability(const FalcoController::CollisionPoint& point){
// //   // traversability will be defined as a [0,1] score (0-untraversable, 1-traversable)

// //   const float& height = point.z;
// //   const float& normal_z = point.normal_z;

// //   // Slope
// //   float slope_score = 1.0 - std::fabs(std::acos(normal_z) / M_PI); // Normalized by pi/2 so the value lies [0-1]
// //   if(slope_score < 0){
// //     ROS_ERROR_STREAM_THROTTLE(1, "Slope score computation not valid: normal_z: " << normal_z << ", score: " << slope_score );
// //   }

// //   // Height penalty (valid in base frame)
// //   float height_penalty = 1.0;

// //   // Compute traversability
// //   float traversability = slope_score * height_penalty;

// //   return traversability;
// // }

// // //-------------------------------------------------------------------------------------------------
// // // Visualizations
// // //-------------------------------------------------------------------------------------------------
// // void FalcoController::setupVisualizations(ros::NodeHandle& node_handle)
// // {
// //   free_paths_pub_         = node_handle.advertise<sensor_msgs::PointCloud2>("/field_local_planner/falco/free_paths", 10);
// //   collision_map_pub_      = node_handle.advertise<sensor_msgs::PointCloud2>("/field_local_planner/falco/collision_map", 10);
// // }

// // void FalcoController::publishVisualizations(const std_msgs::Header& header)
// // {
// //   // Publish collision map
// //   sensor_msgs::PointCloud2 collision_map_cloud;
// //   pcl::toROSMsg(*collision_map_in_base_, collision_map_cloud);
// //   collision_map_cloud.header = header;
// //   collision_map_cloud.header.frame_id = parameters_.base_frame_;
// //   collision_map_pub_.publish(collision_map_cloud);

// //   // Publish free paths
// //   sensor_msgs::PointCloud2 free_paths_cloud;
// //   pcl::toROSMsg(*free_paths_, free_paths_cloud);
// //   free_paths_cloud.header = header;
// //   free_paths_cloud.header.frame_id = parameters_.base_frame_;
// //   free_paths_pub_.publish(free_paths_cloud);
// // }
