#include <math.h>
#include <yaml-cpp/yaml.h>
#include <local_planners_drs/local_planners/rmp.hpp>

namespace local_planners_drs {

Rmp::Rmp() : BaseLocalPlanner() {}

void Rmp::initialize(const Rmp::Parameters& parameters) {
  parameters_ = parameters;
  prepareControlPoints();
}

Twist Rmp::computeTwist() {
  // Get pose in map frame
  T_m_b_ = T_m_f_ * T_f_b_;

  // Cnvert 3D poses to SE(2)
  T_f_b_SE2_ = Pose2(T_f_b_.translation().x(), T_f_b_.translation().y(), T_f_b_.rotation().yaw());

  T_m_b_SE2_ = Pose2(T_m_b_.translation().x(), T_m_b_.translation().y(), T_m_b_.rotation().yaw());

  T_f_g_SE2_ = Pose2(T_f_g_.translation().x(), T_f_g_.translation().y(), T_f_g_.rotation().yaw());

  // Compute RMPs
  computeOptimalAcceleration();

  // Fill twist
  Twist twist;
  twist(0) = 0.0;  // Angular x
  twist(1) = 0.0;  // Angular y
  twist(2) = optimal_velocity_.z();
  // Linear
  twist(3) = optimal_velocity_.x();
  twist(4) = optimal_velocity_.y();
  twist(5) = 0.0;  // Linear z

  return twist;
}

//-------------------------------------------------------------------------------------------------
// RMP optimization
//-------------------------------------------------------------------------------------------------

void Rmp::computeOptimalAcceleration() {
  // ------------------------------------------------------------------------------------
  // Initialize
  // ------------------------------------------------------------------------------------

  // Create problem
  gtsam::Symbol acc_se2_key = gtsam::symbol_shorthand::X(0);
  gtsam::Symbol acc_diff_key = gtsam::symbol_shorthand::X(1);
  Vector3_ acc_se2(acc_se2_key);    // Full SE(2) acceleration
  Vector2_ acc_diff(acc_diff_key);  // Differentially constrained acceleration

  // Create general RMP problem
  RmpProblem problem;

  // Create initial values
  // We use the last optimal acceleration
  Values initial_se2_acc;
  initial_se2_acc.insert(acc_se2_key, optimal_acc_);

  Values initial_diff_acc;
  initial_diff_acc.insert(acc_diff_key, Vector2(optimal_acc_(0), optimal_acc_(2)));

  // Create differential model (if required)
  DifferentialModel motion_model(0.0);
  DifferentialModel_ motion_model_(motion_model);

  // Clear visualizations
  vis_accelerations_.clear();

  Eigen::Rotation2Dd flip_front_direction(0);
  double flip_front_angle = 1.0;
  // if (parameters_.back_is_front_)
  // {
  //     flip_front_direction = Eigen::Rotation2Dd(M_PI);
  //     flip_front_angle = -1;
  // }

  // ------------------------------------------------------------------------------------
  // Add RMPs for control points
  // ------------------------------------------------------------------------------------
  for (size_t i = 0; i < control_points_.size(); i++) {
    // Create expressions for control points
    ControlPoint2 cp(Pose2(), control_points_.at(i).point);
    ControlPoint2_ cp_(cp);

    // Store coordinates in fixed frame
    Vector2 cp_in_map_frame = T_m_b_SE2_ * cp.point();
    control_points_.at(i).point_fixed = cp_in_map_frame;
    float radius = control_points_.at(i).radius;

    // Obstacles
    {
      // Get distance to obstacle and gradient in fixed frame
      double obstacle_distance, obstacle_grad_x, obstacle_grad_y;
      try {
        obstacle_distance = grid_map_.atPosition("sdf", cp_in_map_frame) - radius;
        obstacle_grad_x = grid_map_.atPosition("sdf_gradient_x", cp_in_map_frame);
        obstacle_grad_y = grid_map_.atPosition("sdf_gradient_y", cp_in_map_frame);
      } catch (const std::out_of_range& oor) {
        std::cout << "Error while trying to access [sdf] layer at " << cp.point().transpose() << "\n Error: \n" << oor.what() << std::endl;
      }
      Vector2 gradient_in_base_frame = T_m_b_SE2_.rotation().inverse() * Vector2(obstacle_grad_x, obstacle_grad_y);

      // Create Obstacle RMP
      SdfObstaclePointRmp obstacle_rmp(cp.point(), cp.apply(velocity_2d_), obstacle_distance, gradient_in_base_frame,
                                       parameters_.obstacle_gain, parameters_.obstacle_offset, parameters_.obstacle_steepness,
                                       parameters_.obstacle_weight);
      AccelerationVisualization acc;
      acc.id = "obstacle_" + control_points_.at(i).id;
      acc.acc = flip_front_direction * obstacle_rmp.acceleration().head<2>();
      acc.metric = obstacle_rmp.metric().block(0, 0, 2, 2);
      acc.point = flip_front_direction * control_points_.at(i).point;
      acc.color = Vector3(1.0, 0.0, 0.0);
      acc.weight = parameters_.obstacle_weight;
      vis_accelerations_.push_back(acc);

      // Add RMP
      if (control_points_.at(i).affected_by_obstacle) {
        // if (parameters_.differential_drive)
        // {
        //     problem.addRmp(
        //         applyControlPoint2(cp_, applyDifferentialModel(motion_model_, acc_diff)),
        //         obstacle_rmp);
        // }
        // else
        // {
        problem.addRmp(applyControlPoint2(cp_, acc_se2), obstacle_rmp);
        // }
      }
    }  // end obstacles
  }

  // ------------------------------------------------------------------------------------
  // Add other RMPs to general problem
  // ------------------------------------------------------------------------------------
  // Goal
  // Get distance to goal and gradient in fixed frame
  double goal_distance, goal_grad_x, goal_grad_y;
  gtsam::Vector2 base_position = T_m_b_SE2_.translation();
  try {
    goal_distance = grid_map_.atPosition("geodesic", base_position);
    goal_grad_x = grid_map_.atPosition("geodesic_gradient_x", base_position);
    goal_grad_y = grid_map_.atPosition("geodesic_gradient_y", base_position);

  } catch (const std::out_of_range& oor) {
    std::cout << "Error while trying to access [geodesic] layer at " << base_position.transpose() << "\n Error: \n"
              << oor.what() << std::endl;
  }
  // Create Goal RMP
  Vector2 gradient_in_base_frame = T_m_b_SE2_.rotation().inverse() * Vector2(goal_grad_x, goal_grad_y);

  GeodesicGoal2DRmp geodesic_goal_rmp(T_m_b_SE2_, velocity_2d_, goal_distance, gradient_in_base_frame, parameters_.geodesic_goal_gain,
                                      parameters_.geodesic_goal_offset, parameters_.geodesic_goal_steepness,
                                      parameters_.geodesic_goal_weight);
  AccelerationVisualization acc;
  acc.id = "geodesic_goal";
  acc.acc = flip_front_direction * geodesic_goal_rmp.acceleration().head<2>();
  acc.angular_acc = flip_front_angle * geodesic_goal_rmp.acceleration()(2);
  acc.metric = geodesic_goal_rmp.metric().block(0, 0, 2, 2);
  acc.point = Vector2(0.0, 0.0);
  acc.color = Vector3(1.0, 1.0, 0.0);
  acc.weight = parameters_.geodesic_goal_weight;
  vis_accelerations_.push_back(acc);

  // Create Geodesic Alignment RMP
  GeodesicFlowHeadingRmp geodesic_heading_rmp(T_m_b_SE2_, velocity_2d_, goal_distance, gradient_in_base_frame,
                                              parameters_.geodesic_heading_gain, parameters_.geodesic_heading_offset,
                                              parameters_.geodesic_heading_steepness, parameters_.geodesic_heading_weight);
  AccelerationVisualization acc_alignment;
  acc_alignment.id = "geodesic_heading";
  acc_alignment.acc = flip_front_direction * geodesic_heading_rmp.acceleration().head<2>();
  acc_alignment.angular_acc = flip_front_angle * geodesic_heading_rmp.acceleration()(2);
  acc_alignment.metric = geodesic_heading_rmp.metric().block(0, 0, 2, 2);
  acc_alignment.point = Vector2(0.0, 0.0);
  acc_alignment.color = Vector3(1.0, 1.0, 0.0);
  acc_alignment.weight = parameters_.geodesic_heading_weight;
  vis_accelerations_.push_back(acc_alignment);

  // Goal Position RMP (Base)
  Goal2DRmp goal_position_rmp(T_f_b_SE2_, velocity_2d_, T_f_g_SE2_, parameters_.goal_position_gain, parameters_.goal_position_offset,
                              parameters_.goal_position_steepness, parameters_.goal_position_weight);
  AccelerationVisualization acc_goal_position;
  acc_goal_position.id = "goal_position";
  acc_goal_position.acc = flip_front_direction * goal_position_rmp.acceleration().head<2>();
  acc_goal_position.angular_acc = flip_front_angle * goal_position_rmp.acceleration()(2);
  acc_goal_position.metric = goal_position_rmp.metric().block(0, 0, 2, 2);
  acc_goal_position.point = Vector2(0.0, 0.0);
  acc_goal_position.color = Vector3(1.0, 0.5, 0.0);
  acc_goal_position.weight = parameters_.goal_position_weight;
  vis_accelerations_.push_back(acc_goal_position);

  // Goal Heading RMP (Base)
  GoalHeadingRmp goal_heading_rmp(T_f_b_SE2_, velocity_2d_, T_f_g_SE2_, parameters_.goal_heading_gain, parameters_.goal_heading_offset,
                                  parameters_.goal_heading_steepness, parameters_.goal_heading_weight);
  AccelerationVisualization acc_goal_heading;
  acc_goal_heading.id = "goal_heading";
  acc_goal_heading.acc = flip_front_direction * goal_heading_rmp.acceleration().head<2>();
  acc_goal_heading.angular_acc = flip_front_angle * goal_heading_rmp.acceleration()(2);
  acc_goal_heading.metric = goal_heading_rmp.metric().block(0, 0, 2, 2);
  acc_goal_heading.point = Vector2(0.0, 0.0);
  acc_goal_heading.color = Vector3(1.0, 0.5, 0.0);
  acc_goal_heading.weight = parameters_.goal_heading_weight;
  vis_accelerations_.push_back(acc_goal_heading);

  // Damping RMP
  DampingRmp damping_rmp(T_f_b_SE2_, velocity_2d_, parameters_.damping, parameters_.damping_weight);
  AccelerationVisualization acc_damping;
  acc_damping.id = "damping";
  acc_damping.acc = flip_front_direction * damping_rmp.acceleration().head<2>();
  acc_damping.angular_acc = flip_front_angle * damping_rmp.acceleration()(2);
  acc_damping.metric = damping_rmp.metric().block(0, 0, 2, 2);
  acc_damping.point = Vector2(0.0, 0.0);
  acc_damping.color = Vector3(0.0, 0.0, 1.0);
  acc_damping.weight = parameters_.damping_weight;
  vis_accelerations_.push_back(acc_damping);

  // // Heading RMP
  // double goal_distance = grid_map_.atPosition("geodesic", gtsam_current_pose_in_fixed_.translation());
  HeadingRmp heading_rmp(T_f_b_SE2_, velocity_2d_, Vector2(1.0, 0.0), goal_distance, parameters_.heading_gain, parameters_.heading_offset,
                         parameters_.heading_steepness, parameters_.heading_weight);
  AccelerationVisualization acc_heading;
  acc_heading.id = "heading";
  acc_heading.acc = flip_front_direction * heading_rmp.acceleration().head<2>();
  acc_heading.angular_acc = flip_front_angle * heading_rmp.acceleration()(2);
  acc_heading.metric = heading_rmp.metric().block(0, 0, 2, 2);
  acc_heading.point = Vector2(0.0, 0.0);
  acc_heading.color = Vector3(0.0, 1.0, 0.0);
  acc_heading.weight = parameters_.heading_weight;
  vis_accelerations_.push_back(acc_heading);

  // Add RMPs
  // if (parameters_.differential_drive_)
  // {
  //     // Geodesic goal RMP
  //     problem.addRmp(applyDifferentialModel(motion_model_, acc_diff), geodesic_goal_rmp);

  //     // Geodesic heading RMP
  //     problem.addRmp(applyDifferentialModel(motion_model_, acc_diff), geodesic_heading_rmp);

  //     // Goal Position RMP
  //     problem.addRmp(applyDifferentialModel(motion_model_, acc_diff), goal_position_rmp);

  //     // Goal Heading RMP
  //     problem.addRmp(applyDifferentialModel(motion_model_, acc_diff), goal_heading_rmp);

  //     // Regularization RMP
  //     Vector2 optimal_acc_diff = Vector2::Zero();
  //     optimal_acc_diff(0) = optimal_acc_(0);
  //     optimal_acc_diff(1) = optimal_acc_(2);
  //     problem.addRmp(acc_diff,
  //                    Regularization2Rmp(optimal_acc_diff, parameters_.regularization_));

  //     // Damping RMP
  //     problem.addRmp(applyDifferentialModel(motion_model_, acc_diff), damping_rmp);

  //     // Heading RMP
  //     problem.addRmp(applyDifferentialModel(motion_model_, acc_diff), heading_rmp);
  // }
  // else
  // {
  // Geodesic goal RMP
  problem.addRmp(acc_se2, geodesic_goal_rmp);

  // Geodesic heading RMP
  problem.addRmp(acc_se2, geodesic_heading_rmp);

  // Goal Position RMP
  problem.addRmp(acc_se2, goal_position_rmp);

  // Goal Heading RMP
  problem.addRmp(acc_se2, goal_heading_rmp);

  // Regularization RMP
  problem.addRmp(acc_se2, Regularization3Rmp(optimal_acc_, parameters_.regularization));

  // Damping RMP
  problem.addRmp(acc_se2, damping_rmp);

  // Heading RMP
  problem.addRmp(acc_se2, heading_rmp);
  // }
  // profiler_ptr_->endEvent("1.3.1.prepare_rmps");

  // ------------------------------------------------------------------------------------
  // Solve RMP general problem
  // ------------------------------------------------------------------------------------
  // profiler_ptr_->startEvent("1.3.2.solve_rmps");
  Values solution;
  bool valid_solution = true;
  try {
    // if (parameters_.differential_drive_)
    // {
    //     solution = problem.solve(initial_diff_acc, true);
    // }
    // else
    // {
    solution = problem.solve(initial_se2_acc, true);
    // }
  } catch (const std::exception& e) {
    std::cout << "Error thrown during optimization of RMP problem:\n" << std::string(e.what()) << std::endl;
    optimal_acc_ = Vector3::Zero();
    optimal_metric_ = Matrix3::Identity();
    valid_solution = false;
  }

  // Extract solution
  if (valid_solution) {
    // if (parameters_.differential_drive_)
    // {
    //     Vector2 optimal_acc_diff = solution.at<Vector2>(acc_diff_key);
    //     Matrix2 optimal_metric_diff = problem.metric();

    //     // Fill optimal acceleration
    //     optimal_acc_(0) = optimal_acc_diff(0);
    //     optimal_acc_(1) = 0.0;
    //     optimal_acc_(2) = optimal_acc_diff(1);

    //     // Fill optimal metric
    //     optimal_metric_ = Eigen::Matrix3d::Identity();
    //     optimal_metric_(0, 0) = optimal_metric_diff(0, 0);
    //     optimal_metric_(0, 2) = optimal_metric_diff(0, 1);
    //     optimal_metric_(2, 0) = optimal_metric_diff(1, 0);
    //     optimal_metric_(2, 2) = optimal_metric_diff(1, 1);
    // }
    // else
    // {
    optimal_acc_ = solution.at<Vector3>(acc_se2_key);
    optimal_metric_ = problem.metric();
    // }
  } else {
    std::cout << "Invalid solution" << std::endl;
  }

  // profiler_ptr_->endEvent("1.3.2.solve_rmps");

  // ------------------------------------------------------------------------------------
  // Integrate acceleration
  // ------------------------------------------------------------------------------------
  optimal_velocity_ = optimal_acc_ * parameters_.integration_time;
}

//-------------------------------------------------------------------------------------------------
// Control points
//-------------------------------------------------------------------------------------------------
void Rmp::prepareControlPoints() {
  control_points_.clear();

  // Open YAML file
  YAML::Node cp_config = YAML::LoadFile(parameters_.config_folder + "control_points.yaml");

  // Preallocate robot dimensions
  double robot_half_length = (parameters_.robot_length + parameters_.robot_clearance) / 2;
  double robot_half_width = (parameters_.robot_width + parameters_.robot_clearance) / 2;

  // Read the YAML file
  for (YAML::const_iterator it = cp_config["control_points"].begin(); it != cp_config["control_points"].end(); ++it) {
    ControlPointStruct control_point;
    // Id
    control_point.id = it->first.as<std::string>();

    YAML::Node cp_attributes = it->second.as<YAML::Node>();
    // Point
    std::vector<double> factors = utils::get<std::vector<double>>(cp_attributes, "point_factor", std::vector<double>());
    control_point.point = Vector2(robot_half_length * factors[0], robot_half_width * factors[1]);
    // Color
    std::vector<double> rgb = utils::get<std::vector<double>>(cp_attributes, "color", std::vector<double>());
    control_point.color = Vector3(rgb[0], rgb[1], rgb[2]);
    // Accelerations
    control_point.acc = Vector2::Zero();
    // Metric
    control_point.metric = Matrix2::Identity();
    // Affected by forces
    control_point.affected_by_goal = utils::get<bool>(cp_attributes, "affected_by_goal", true);
    control_point.affected_by_obstacle = utils::get<bool>(cp_attributes, "affected_by_obstacle", true);
    // Radius
    control_point.radius = std::min(parameters_.robot_length, parameters_.robot_width) * 0.5 * parameters_.sphere_radius_factor;

    // Add to control points
    control_points_.push_back(control_point);
  }
}

}  // namespace local_planners_drs

// //-------------------------------------------------------------------------------------------------
// // Constructor and parameters
// //-------------------------------------------------------------------------------------------------
// RmpController::RmpController(const ControllerParameters& params) : ControllerBase(params)
// {
//     node_handle_ = ros::NodeHandle("~rmp");

//     // Update parameters
//     ROS_WARN("[ControllerBase] Setting up Controller Parameters");
//     updateControllerParameters(node_handle_);

//     // Setup Dynamic reconfigure
//     ROS_WARN("[ControllerBase] Setting up Dynamic reconfigure");
//     setupDynamicReconfigureServer(node_handle_);

//     // Setup visualizations
//     ROS_WARN("[ControllerBase] Setting up Visualizations");
//     setupVisualizations(node_handle_);
// };

// void RmpController::updateControllerParameters(ros::NodeHandle& node_handle)
// {
//     parameters_.config_folder_ =
//         ros::package::getPath("local_planners_drs") + "/config/controllers/rmp/";

//     // RMP params
//     parameters_.sphere_radius_factor_ =
//         utils::getParameterDefault<double>(node_handle, "sphere_radius_factor", 1.0);
//     parameters_.geodesic_goal_gain_ =
//         utils::getParameterDefault<double>(node_handle, "geodesic_goal_gain", 7.0);
//     parameters_.geodesic_goal_offset_ =
//         utils::getParameterDefault<double>(node_handle, "geodesic_goal_offset", 0.5);
//     parameters_.geodesic_goal_steepness_ =
//         utils::getParameterDefault<double>(node_handle, "geodesic_goal_steepness", 1.0);

//     parameters_.geodesic_heading_gain_ =
//         utils::getParameterDefault<double>(node_handle, "geodesic_heading_gain", 7.0);
//     parameters_.geodesic_heading_offset_ =
//         utils::getParameterDefault<double>(node_handle, "geodesic_heading_offset", 0.5);
//     parameters_.geodesic_heading_steepness_ =
//         utils::getParameterDefault<double>(node_handle, "geodesic_heading_steepness", 1.0);

//     parameters_.goal_position_gain_ =
//         utils::getParameterDefault<double>(node_handle, "goal_position_gain", 7.0);
//     parameters_.goal_position_offset_ =
//         utils::getParameterDefault<double>(node_handle, "goal_position_offset", 0.5);
//     parameters_.goal_position_steepness_ =
//         utils::getParameterDefault<double>(node_handle, "goal_position_steepness", 1.0);

//     parameters_.goal_heading_gain_ =
//         utils::getParameterDefault<double>(node_handle, "goal_heading_gain", 7.0);
//     parameters_.goal_heading_offset_ =
//         utils::getParameterDefault<double>(node_handle, "goal_heading_offset", 0.5);
//     parameters_.goal_heading_steepness_ =
//         utils::getParameterDefault<double>(node_handle, "goal_heading_steepness", 1.0);

//     parameters_.damping_ = utils::getParameterDefault<double>(node_handle, "damping", 1.0);
//     parameters_.obstacle_gain_ =
//         utils::getParameterDefault<double>(node_handle, "obstacle_gain", 1.0);
//     parameters_.obstacle_offset_ =
//         utils::getParameterDefault<double>(node_handle, "obstacle_offset", 0.1);
//     parameters_.obstacle_steepness_ =
//         utils::getParameterDefault<double>(node_handle, "obstacle_steepness", 1.0);

//     parameters_.heading_gain_ =
//         utils::getParameterDefault<double>(node_handle, "heading_gain", 1.0);
//     parameters_.heading_offset_ =
//         utils::getParameterDefault<double>(node_handle, "heading_offset", 0.5);
//     parameters_.heading_steepness_ =
//         utils::getParameterDefault<double>(node_handle, "heading_steepness", 1.0);
//     parameters_.regularization_ =
//         utils::getParameterDefault<double>(node_handle, "acceleration_regularization", 0.1);
//     parameters_.integration_time_ =
//         utils::getParameterDefault<double>(node_handle, "integration_time", 0.3);

//     parameters_.geodesic_goal_weight_ =
//         utils::getParameterDefault<double>(node_handle, "geodesic_goal_weight", 1.0);
//     parameters_.geodesic_heading_weight_ =
//         utils::getParameterDefault<double>(node_handle, "geodesic_heading_weight", 1.0);

//     parameters_.goal_position_weight_ =
//         utils::getParameterDefault<double>(node_handle, "goal_position_weight", 1.0);
//     parameters_.goal_heading_weight_ =
//         utils::getParameterDefault<double>(node_handle, "goal_heading_weight", 1.0);

//     parameters_.damping_weight_ =
//         utils::getParameterDefault<double>(node_handle, "damping_weight", 1.0);
//     parameters_.obstacle_weight_ =
//         utils::getParameterDefault<double>(node_handle, "obstacle_weight", 1.0);
//     parameters_.heading_weight_ =
//         utils::getParameterDefault<double>(node_handle, "heading_weight", 1.0);
//     parameters_.check_internal_collisions_ =
//         utils::getParameterDefault<bool>(node_handle, "check_internal_collisions", false);

//     // Prepare control points
//     prepareControlPoints();
// }

// void RmpController::setupDynamicReconfigureServer(ros::NodeHandle& node_handle)
// {
//     // Create Dynamic Reconfigure server passing correct node handle
//     dynamic_reconfigure_server_ = std::make_shared<
//         dynamic_reconfigure::Server<local_planners_drs::RmpControllerConfig>>(node_handle);

//     dynamic_reconfigure_callback_ =
//         boost::bind(&RmpController::dynamicReconfigureCallback, this, _1, _2);
//     dynamic_reconfigure_server_->setCallback(dynamic_reconfigure_callback_);
// }

// void RmpController::dynamicReconfigureCallback(
//     local_planners_drs::RmpControllerConfig& config, uint32_t level)
// {
//     // RMP parameters
//     utils::assignAndPrintDiff("sphere_radius_factor", parameters_.sphere_radius_factor_,
//                               config.sphere_radius_factor);
//     utils::assignAndPrintDiff("geodesic_goal_gain", parameters_.geodesic_goal_gain_,
//                               config.geodesic_goal_gain);
//     utils::assignAndPrintDiff("geodesic_goal_offset", parameters_.geodesic_goal_offset_,
//                               config.geodesic_goal_offset);
//     utils::assignAndPrintDiff("geodesic_goal_steepness", parameters_.geodesic_goal_steepness_,
//                               config.geodesic_goal_steepness);

//     utils::assignAndPrintDiff("geodesic_heading_gain", parameters_.geodesic_heading_gain_,
//                               config.geodesic_heading_gain);
//     utils::assignAndPrintDiff("geodesic_heading_offset", parameters_.geodesic_heading_offset_,
//                               config.geodesic_heading_offset);
//     utils::assignAndPrintDiff("geodesic_heading_steepness", parameters_.geodesic_heading_steepness_,
//                               config.geodesic_heading_steepness);

//     utils::assignAndPrintDiff("goal_position_gain", parameters_.goal_position_gain_,
//                               config.goal_position_gain);
//     utils::assignAndPrintDiff("goal_position_offset", parameters_.goal_position_offset_,
//                               config.goal_position_offset);
//     utils::assignAndPrintDiff("goal_position_steepness",
//                               parameters_.goal_position_steepness_,
//                               config.goal_position_steepness);

//     utils::assignAndPrintDiff("goal_heading_gain", parameters_.goal_heading_gain_,
//                               config.goal_heading_gain);
//     utils::assignAndPrintDiff("goal_heading_offset", parameters_.goal_heading_offset_,
//                               config.goal_heading_offset);
//     utils::assignAndPrintDiff("goal_heading_steepness",
//                               parameters_.goal_heading_steepness_,
//                               config.goal_heading_steepness);

//     utils::assignAndPrintDiff("damping", parameters_.damping_, config.damping);
//     utils::assignAndPrintDiff("obstacle_gain", parameters_.obstacle_gain_,
//                               config.obstacle_gain);
//     utils::assignAndPrintDiff("obstacle_offset", parameters_.obstacle_offset_,
//                               config.obstacle_offset);
//     utils::assignAndPrintDiff("obstacle_steepness", parameters_.obstacle_steepness_,
//                               config.obstacle_steepness);

//     utils::assignAndPrintDiff("heading_gain", parameters_.heading_gain_,
//                               config.heading_gain);
//     utils::assignAndPrintDiff("heading_offset", parameters_.heading_offset_,
//                               config.heading_offset);
//     utils::assignAndPrintDiff("heading_steepness", parameters_.heading_steepness_,
//                               config.heading_steepness);

//     utils::assignAndPrintDiff("acceleration_regularization", parameters_.regularization_,
//                               config.acceleration_regularization);
//     utils::assignAndPrintDiff("integration_time", parameters_.integration_time_,
//                               config.integration_time);

//     utils::assignAndPrintDiff("geodesic_goal_weight", parameters_.geodesic_goal_weight_,
//                               config.geodesic_goal_weight);
//      utils::assignAndPrintDiff("geodesic_heading_weight", parameters_.geodesic_heading_weight_,
//                               config.geodesic_heading_weight);
//     utils::assignAndPrintDiff("goal_position_weight", parameters_.goal_position_weight_,
//                               config.goal_position_weight);
//     utils::assignAndPrintDiff("goal_heading_weight", parameters_.goal_heading_weight_,
//                               config.goal_heading_weight);

//     utils::assignAndPrintDiff("damping_weight", parameters_.damping_weight_,
//                               config.damping_weight);
//     utils::assignAndPrintDiff("obstacle_weight", parameters_.obstacle_weight_,
//                               config.obstacle_weight);
//     utils::assignAndPrintDiff("heading_weight", parameters_.heading_weight_,
//                               config.heading_weight);
//     utils::assignAndPrintDiff("check_internal_collisions",
//                               parameters_.check_internal_collisions_,
//                               config.check_internal_collisions);

//     // Prepare control points
//     prepareControlPoints();
// }

// //-------------------------------------------------------------------------------------------------
// // Controller methods
// //-------------------------------------------------------------------------------------------------
// void RmpController::customPreProcessLocalMap() {}

// //-------------------------------------------------------------------------------------------------
// // Controller methods
// //-------------------------------------------------------------------------------------------------
// void RmpController::customPreProcessController()
// {
//     // Reset flags
//     is_colliding_ = false;

//     // Fill robot velocity
//     velocity_2d_.x() = b_v_b_t_(3);  // x component
//     velocity_2d_.y() = b_v_b_t_(4);  // y component
//     velocity_2d_.z() = b_v_b_t_(2);  // angular z component

//     ROS_INFO_STREAM_THROTTLE(1, "last optimal acc: " << optimal_acc_.transpose());
//     ROS_INFO_STREAM_THROTTLE(1, "last optimal vel: " << optimal_velocity_.transpose());
//     ROS_INFO_STREAM_THROTTLE(1, "Current velocity: " << velocity_2d_.transpose());

//     // Compute pose in local map frame, since it's used to query the grid map
//     std::string map_frame = grid_map_.getFrameId();
//     tf::StampedTransform fixed_to_map_transform;
//     Eigen::Isometry3d fixed_to_map;
//     try
//     {
//         // Listen transform
//         tf_listener_.lookupTransform(map_frame, parameters_.fixed_frame_, ros::Time(0),
//                                      fixed_to_map_transform);
//         // Convert to Isometry3d
//         fixed_to_map = Eigen::Isometry3d::Identity();
//         tf::transformTFToEigen(fixed_to_map_transform, fixed_to_map);

//         // Update current pose
//         current_pose_in_map_ = fixed_to_map * f_T_fb_t_;
//     }
//     catch (tf::TransformException& ex)
//     {
//         ROS_ERROR_STREAM("tf listener failed to obtain pose T_" << map_frame << "_"
//                                                                 << parameters_.fixed_frame_);
//         ROS_ERROR("%s", ex.what());
//         return;
//     }

//     // Create gtsam poses
//     gtsam_current_pose_in_fixed_ =
//         gtsam::Pose2(f_T_fb_t_2d_.translation().x(), f_T_fb_t_2d_.translation().y(),
//                      Eigen::Rotation2Dd(f_T_fb_t_2d_.linear()).angle());

//     Eigen::AngleAxisd axis_angle_pose_in_map(current_pose_in_map_.rotation());
//     double pose_in_map_angle = axis_angle_pose_in_map.axis()(2) * axis_angle_pose_in_map.angle();
//     gtsam_current_pose_in_map_ =
//         gtsam::Pose2(current_pose_in_map_.translation().x(), current_pose_in_map_.translation().y(),
//                      pose_in_map_angle);

//     Eigen::AngleAxisd axis_angle(f_Tgoal_fb_t_.rotation());
//     double goal_angle = axis_angle.axis()(2) * axis_angle.angle();
//     gtsam_current_goal_in_fixed_ =
//         gtsam::Pose2(f_Tgoal_fb_t_.translation().x(), f_Tgoal_fb_t_.translation().y(), goal_angle);

//     // Compute optimal acceleration
//     profiler_ptr_->startEvent("1.3.compute_rmp_acc");
//     computeOptimalAcceleration();
//     profiler_ptr_->endEvent("1.3.compute_rmp_acc");
// }

// ControllerBase::OutputAction RmpController::computeCommandGoalBehind(
//     Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d& output_angular_velocity)
// {
//     // Compute angular velocity command
//     output_linear_velocity(0) = 0.0;
//     output_linear_velocity(1) = 0.0;
//     output_linear_velocity(2) = 0.0;

//     // compute angular velocity only command
//     output_angular_velocity(0) = 0.0;
//     output_angular_velocity(1) = 0.0;
//     output_angular_velocity(2) = 0.0;

//     return ControllerBase::OutputAction::SEND_COMMAND;
// }

// ControllerBase::OutputAction RmpController::computeCommandTurnToGoal(
//     Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d& output_angular_velocity)
// {
//     return computeCommandForward(output_linear_velocity, output_angular_velocity);
// }

// ControllerBase::OutputAction RmpController::computeCommandForward(
//     Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d& output_angular_velocity)
// {
//     output_linear_velocity(0) = optimal_velocity_.x();
//     output_linear_velocity(1) = optimal_velocity_.y();
//     output_angular_velocity(2) = optimal_velocity_.z();

//     return ControllerBase::OutputAction::SEND_COMMAND;
// }

// ControllerBase::OutputAction RmpController::computeCommandTurnToDestination(
//     Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d& output_angular_velocity)
// {
//     return computeCommandForward(output_linear_velocity, output_angular_velocity);
// }

// ControllerBase::OutputAction RmpController::computeCommandFinished(
//     Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d& output_angular_velocity)
// {
//     ControllerBase::OutputAction action = ControllerBase::OutputAction::SEND_COMMAND;

//     if (goals_.size() > 0)
//     {
//         // ROS_INFO("Continuing to walk without stopping");
//         action = SEND_NOTHING;
//     }

//     velocity_2d_ = gtsam::Vector3::Zero();

//     return action;
// }

// void RmpController::computeCommandPostProcess(ControllerBase::OutputAction& action,
//                                               Eigen::Vector3d& output_linear_velocity,
//                                               Eigen::Vector3d& output_angular_velocity,
//                                               std::vector<Eigen::Vector3d>& path_to_goal)
// {
//     path_to_goal.clear();

//     // Integrate the velocity to get a path
//     const double T = 2;  // seconds
//     for (double dt = 0; dt < T; dt += 0.1)
//     {
//         gtsam::Pose2 pose_t = gtsam::Pose2::Expmap(optimal_velocity_ * dt);

//         path_to_goal.push_back(Eigen::Vector3d(pose_t.x(), pose_t.y(), 0.0));
//     }
// }

// //-------------------------------------------------------------------------------------------------
// // Visualizations
// //-------------------------------------------------------------------------------------------------
// void RmpController::setupVisualizations(ros::NodeHandle& node_handle)
// {
//     control_points_pub_ = node_handle.advertise<visualization_msgs::MarkerArray>(
//         "/local_planners_drs/rmp/control_points", 10);
//     accelerations_pub_ = node_handle.advertise<visualization_msgs::MarkerArray>(
//         "/local_planners_drs/rmp/accelerations", 10);
// }

// void RmpController::publishVisualizations(const std_msgs::Header& header)
// {
//     // Publish Control points (base)
//     visualization_msgs::MarkerArray control_points_array =
//         controlPointsToMarkerArray(control_points_, header);
//     control_points_pub_.publish(control_points_array);

//     // // Publish accelerations
//     visualization_msgs::MarkerArray accs_array =
//         accelerationsToMarkerArray(vis_accelerations_, header);
//     accelerations_pub_.publish(accs_array);
// }

// visualization_msgs::MarkerArray RmpController::controlPointsToMarkerArray(
//     const std::vector<ControlPointStruct>& control_points, const std_msgs::Header& header)
// {
//     int i = 0;
//     visualization_msgs::MarkerArray cp_vis;
//     for (auto cp : control_points)
//     {
//         // Control point marker
//         visualization_msgs::Marker marker_cp;
//         marker_cp.header = header;
//         marker_cp.header.frame_id = parameters_.base_frame_;
//         marker_cp.ns = "control_point";
//         marker_cp.id = i;
//         marker_cp.type = visualization_msgs::Marker::SPHERE;
//         marker_cp.action = visualization_msgs::Marker::ADD;
//         marker_cp.pose.position.x = cp.point_.x();
//         marker_cp.pose.position.y = cp.point_.y();
//         marker_cp.pose.position.z = 0.0;
//         marker_cp.pose.orientation.x = 0.0;
//         marker_cp.pose.orientation.y = 0.0;
//         marker_cp.pose.orientation.z = 0.0;
//         marker_cp.pose.orientation.w = 1.0;
//         marker_cp.scale.x = 2 * cp.radius_;
//         marker_cp.scale.y = 2 * cp.radius_;
//         marker_cp.scale.z = 2 * cp.radius_;
//         marker_cp.color.r = cp.color_(0);
//         marker_cp.color.g = cp.color_(1);
//         marker_cp.color.b = cp.color_(2);
//         marker_cp.color.a = 0.2;
//         // Add marker
//         cp_vis.markers.push_back(marker_cp);

//         // Update ids
//         i++;
//     }

//     return cp_vis;
// }

// visualization_msgs::MarkerArray RmpController::accelerationsToMarkerArray(
//     const std::vector<AccelerationVisualization>& accelerations, const std_msgs::Header& header)
// {
//     int i = 0;
//     visualization_msgs::MarkerArray acc_vis;

//     for (auto acc : accelerations)
//     {
//         // Accelerations
//         visualization_msgs::Marker marker_acc;
//         {
//             marker_acc.header = header;
//             marker_acc.header.frame_id = parameters_.base_frame_;
//             marker_acc.ns = "acc_" + acc.id_;
//             marker_acc.id = i;
//             marker_acc.type = visualization_msgs::Marker::ARROW;
//             marker_acc.action = visualization_msgs::Marker::ADD;
//             marker_acc.pose.position.x = acc.point_.x();
//             marker_acc.pose.position.y = acc.point_.y();
//             marker_acc.pose.position.z = 0.0;
//             marker_acc.pose.orientation.x = 0.0;
//             marker_acc.pose.orientation.y = 0.0;
//             marker_acc.pose.orientation.z = 0.0;
//             marker_acc.pose.orientation.w = 1.0;
//             marker_acc.scale.x = 0.02;
//             marker_acc.scale.y = 0.05;
//             marker_acc.scale.z = 0.05;
//             marker_acc.color.r = acc.color_(0);
//             marker_acc.color.g = acc.color_(1);
//             marker_acc.color.b = acc.color_(2);
//             marker_acc.color.a = 1.0;
//             geometry_msgs::Point p1;
//             p1.x = 0.0;
//             p1.y = 0.0;
//             p1.z = 0.0;
//             geometry_msgs::Point p2;
//             p2.x = acc.acc_.x() * 0.05 * acc.weight_;
//             p2.y = acc.acc_.y() * 0.05 * acc.weight_;
//             p2.z = 0.0;
//             marker_acc.points.push_back(p1);
//             marker_acc.points.push_back(p2);
//         }
//         // Add marker
//         acc_vis.markers.push_back(marker_acc);

//         // Angular accelerations
//         visualization_msgs::Marker marker_ang_acc;
//         {
//             marker_ang_acc.header = header;
//             marker_ang_acc.header.frame_id = parameters_.base_frame_;
//             marker_ang_acc.ns = "angular_acc_" + acc.id_;
//             marker_ang_acc.id = i;
//             marker_ang_acc.type = visualization_msgs::Marker::ARROW;
//             marker_ang_acc.action = visualization_msgs::Marker::ADD;
//             marker_ang_acc.pose.position.x = acc.point_.x();
//             marker_ang_acc.pose.position.y = acc.point_.y();
//             marker_ang_acc.pose.position.z = 0.0;
//             marker_ang_acc.pose.orientation.x = 0.0;
//             marker_ang_acc.pose.orientation.y = 0.0;
//             marker_ang_acc.pose.orientation.z = 0.0;
//             marker_ang_acc.pose.orientation.w = 1.0;
//             marker_ang_acc.scale.x = 0.04;
//             marker_ang_acc.scale.y = 0.07;
//             marker_ang_acc.scale.z = 0.07;
//             marker_ang_acc.color.r = acc.color_(0);
//             marker_ang_acc.color.g = acc.color_(1);
//             marker_ang_acc.color.b = acc.color_(2);
//             marker_ang_acc.color.a = 1.0;
//             geometry_msgs::Point p1;
//             p1.x = 0.0;
//             p1.y = 0.0;
//             p1.z = 0.0;
//             geometry_msgs::Point p2;
//             p2.x = 0.0;
//             p2.y = acc.angular_acc_ * 0.05 * acc.weight_;
//             p2.z = 0.0;
//             marker_ang_acc.points.push_back(p1);
//             marker_ang_acc.points.push_back(p2);
//         }
//         // Add marker
//         acc_vis.markers.push_back(marker_ang_acc);

//         // Metric
//         // To plot the metric we use supereight_atlas' approach
//         // https://github.com/ori-drs/supereight_atlas/blob/master/se_atlas_ros/src/ros/ros_pub.cpp#L1157
//         // We first need to compute the eigenvectors and eigenvalues
//         Eigen::Vector2d vis_eigenvalues(Eigen::Vector2d::Zero());
//         Eigen::Matrix2d vis_eigenvectors(Eigen::Matrix2d::Zero());
//         Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> vis_solver(acc.metric_);

//         if (vis_solver.info() == Eigen::Success)
//         {
//             vis_eigenvalues = vis_solver.eigenvalues();
//             vis_eigenvectors = vis_solver.eigenvectors();
//             // ROS_INFO_STREAM_THROTTLE(1, "eigen vectors: \n" << vis_eigenvectors);
//             // ROS_INFO_STREAM_THROTTLE(1, "eigen values:  \n" << vis_eigenvalues);
//         }
//         else
//         {
//             ROS_WARN_STREAM("Couldn't compute eigenvectors for metric of acc [" << acc.id_ << "]");
//             continue;
//         }
//         // Create rotation matrix from eigenvectors
//         Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
//         rot.block(0, 0, 2, 2) = vis_eigenvectors;

//         Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
//         pose.translate(Eigen::Vector3d(acc.point_.x(), acc.point_.y(), 0.0));
//         pose.rotate(rot);

//         geometry_msgs::Pose metric_pose;
//         tf::poseEigenToMsg(pose, metric_pose);

//         // Create marker
//         visualization_msgs::Marker marker_metric;
//         {
//             marker_metric.header = header;
//             marker_metric.header.frame_id = parameters_.base_frame_;
//             marker_metric.ns = "metrics_" + acc.id_;
//             marker_metric.id = i;
//             marker_metric.type = visualization_msgs::Marker::SPHERE;
//             marker_metric.action = visualization_msgs::Marker::ADD;
//             marker_metric.pose = metric_pose;
//             marker_metric.scale.x = sqrt(vis_eigenvalues[0]) * 1.0;
//             marker_metric.scale.y = sqrt(vis_eigenvalues[1]) * 1.0;
//             marker_metric.scale.z = 0.1;
//             marker_metric.color.r = acc.color_(0);
//             marker_metric.color.g = acc.color_(1);
//             marker_metric.color.b = acc.color_(2);
//             marker_metric.color.a = 0.2;
//         }
//         // Add marker
//         acc_vis.markers.push_back(marker_metric);

//         // Update ids
//         i++;
//     }

//     return acc_vis;
// }