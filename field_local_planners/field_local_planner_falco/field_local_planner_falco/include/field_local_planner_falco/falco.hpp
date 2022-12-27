#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <stdio.h>
#include <stdlib.h>
#include <field_local_planner_base/local_planner.hpp>
#include <field_local_planner_base/utils.hpp>

namespace field_local_planner {

class Falco : public BaseLocalPlanner {
 public:
  struct Parameters : BaseLocalPlanner::Parameters {
    std::string config_folder = "/local_planners/falco/";  // pathFolder

    double goal_clearance = 0.1;  // goalClearRange
    double robot_clearance = 0.1;

    double sensor_range = 2.0;

    // gains
    double linear_gain_p = 1.0;
    double angular_gain_p = 1.0;
    // linear acceleration
    double linear_acceleration = 0.0001;

    // Path scale
    double path_scale = 1.25;  // pathScale

    // adjust path scale
    double path_scale_step = 0.25;  // pathScaleStep
    double min_path_scale = 0.75;   // minPathScale

    double path_range_step = 0.5;  // pathRangeStep
    double min_path_range = 1.0;   // minPathRange

    // command generation
    double look_ahead_distance = 0.5;  // lookAheadDis // Used to select a point in the path to be followed

    // costs
    double differential_mode_cost = 1.0;
    double goal_cost = 1.0;
    double traversability_cost = 1.0;

    // Threshold
    double traversable_thr = 0.5;

    // Used to accept a path as a candidate
    int point_per_path_thr = 2;  // pointPerPathThre

    bool check_rotational_collisions = true;  // checkRotObstacle
    bool use_path_crop_by_goal = false;       // pathCropByGoal
  };

 public:
  Falco();
  void loadParameters(const Parameters& parameters);
  Twist computeTwist();
  Path computePath();

 private:
  // Load paths
  FILE* openFile(std::string& file);
  int readPlyHeader(FILE* file_ptr);
  void loadPathsFromFile(std::string folder);
  void loadPrecomputedPaths(std::string folder);
  void loadPaths(std::string folder);
  void loadPathsList(std::string folder);
  void loadCorrespondences(std::string folder);

  // Specific methods
  void computeFreePath();
  void clearPathsLists();
  void computeMapCollisions();
  void computePathScores();
  bool findBestPath();
  void computeIntermediateCarrot();

 protected:
  Parameters parameters_;

  static constexpr int NUM_PATHS = 343;
  static constexpr int NUM_GROUPS = 7;
  static constexpr int NUM_MAP_CLOUD_STACK = 1;
  static constexpr int NUM_GRID_VOXELS_X = 161;
  static constexpr int NUM_GRID_VOXELS_Y = 451;
  static constexpr int NUM_GRID_VOXELS = NUM_GRID_VOXELS_X * NUM_GRID_VOXELS_Y;
  static constexpr float GRID_VOXEL_SIZE = 0.02;
  static constexpr float SEARCH_RADIUS = 0.45;
  static constexpr float GRID_VOXEL_OFFSET_X = 3.2;
  static constexpr float GRID_VOXEL_OFFSET_Y = 4.5;

  // Internal variables
  double path_scale_;
  double path_range_;
  double robot_diameter_;

  // last linear velocity applied
  double last_linear_velocity_;

  // FALCO
  Vector3 intermediate_carrot_;  // a collision-free intermediate goal before reaching the desired goal
  Vector3 path_end_;             // end of the best chosen path

  // Path allocation
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_stack[NUM_MAP_CLOUD_STACK];
  pcl::PointCloud<pcl::PointXYZ>::Ptr precomputed_paths_[NUM_GROUPS];  // precomputed trajectories

  pcl::PointCloud<pcl::PointXYZI>::Ptr
      paths_[NUM_PATHS];  // visualization: all the precomputed trajectories. They are visualized with different rotations
  pcl::PointCloud<pcl::PointXYZI>::Ptr free_paths_;  // visualization: free paths after collision checking

  std::vector<int> correspondences_[NUM_GRID_VOXELS];  // the number of correspondences is equal to the number of voxels

  // Path validity
  int path_list_[NUM_PATHS] = {0};

  float end_dir_path_list_[NUM_PATHS] = {0};
  int clear_path_list_[36 * NUM_PATHS] = {0};

  float path_likelihood_[36 * NUM_PATHS] = {0};         // The likelihood of each path to reach the goal
  float path_group_likelihood_[36 * NUM_GROUPS] = {0};  // The likelihood of the group to reach the goal

  // The quantities below are used to compute the traversability probability of a path
  float path_traversability_score_[36 * NUM_PATHS] = {0};         // Integrated traversability score along the path
  int path_traversability_crossed_voxels_[36 * NUM_PATHS] = {0};  // Number of voxels crossed by the path

  float path_deformation_list_[36 * NUM_PATHS] = {0};  // Deformation of the path due to the terrain

  // Feasible path found
  bool feasible_path_found_;
  int best_path_group_id_;
  float best_path_group_score_;
  std::vector<Vector3> best_path_;
};

}  // namespace field_local_planner

// class FalcoController : public ControllerBase {

//   // Definitions
//   using CollisionPoint = pcl::PointXYZINormal;

// public:
//   FalcoController(const ControllerParameters& params);

//   // Set parameters
//   void updateControllerParameters(ros::NodeHandle& node_handle);

//   // Visualizations
//   void setupVisualizations(ros::NodeHandle& node_handle);
//   void publishVisualizations(const std_msgs::Header& header);

//   // Dynamic reconfigure
//   void setupDynamicReconfigureServer(ros::NodeHandle& node_handle);
//   void dynamicReconfigureCallback(field_local_planner::FalcoControllerConfig &config, uint32_t level);

// private:

//   // Controller implementation
//   void customPreProcessLocalMap();
//   void customPreProcessController();
//   ControllerBase::OutputAction computeCommandGoalBehind(Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d&
//   output_angular_velocity); ControllerBase::OutputAction computeCommandTurnToGoal(Eigen::Vector3d& output_linear_velocity,
//   Eigen::Vector3d& output_angular_velocity); ControllerBase::OutputAction computeCommandForward(Eigen::Vector3d& output_linear_velocity,
//   Eigen::Vector3d& output_angular_velocity); ControllerBase::OutputAction computeCommandTurnToDestination(Eigen::Vector3d&
//   output_linear_velocity, Eigen::Vector3d& output_angular_velocity); ControllerBase::OutputAction computeCommandFinished(Eigen::Vector3d&
//   output_linear_velocity, Eigen::Vector3d& output_angular_velocity); void computeCommandPostProcess(ControllerBase::OutputAction& action,
//                                  Eigen::Vector3d& output_linear_velocity,
//                                  Eigen::Vector3d& output_angular_velocity,
//                                  std::vector<Eigen::Vector3d>& path_to_goal);

//   // Load paths
//   FILE* openFile(std::string& file);
//   int readPlyHeader(FILE* file_ptr);
//   void loadPathsFromFile(std::string folder);
//   void loadPrecomputedPaths(std::string folder);
//   void loadPaths(std::string folder);
//   void loadPathsList(std::string folder);
//   void loadCorrespondences(std::string folder);

//   // Specific methods
//   void computeFreePath();
//   void clearPathsLists();
//   void computeMapCollisions();
//   void computePathScores();
//   bool findBestPath();
//   void computeIntermediateCarrot();

//   // Preprocess local map
//   void convertElevationMapToCloud();
//   void updateElevationMapFrame();
//   // traversability will be defined as a [0,1] score (0-untraversable, 1-traversable)
//   float computeTraversability(const CollisionPoint& point);;

// private:
//   static constexpr int NUM_PATHS  = 343;
//   static constexpr int NUM_GROUPS = 7;
//   static constexpr int NUM_MAP_CLOUD_STACK = 1;
//   static constexpr int NUM_GRID_VOXELS_X = 161;
//   static constexpr int NUM_GRID_VOXELS_Y = 451;
//   static constexpr int NUM_GRID_VOXELS = NUM_GRID_VOXELS_X * NUM_GRID_VOXELS_Y;
//   static constexpr float GRID_VOXEL_SIZE = 0.02;
//   static constexpr float SEARCH_RADIUS = 0.45;
//   static constexpr float GRID_VOXEL_OFFSET_X = 3.2;
//   static constexpr float GRID_VOXEL_OFFSET_Y = 4.5;

//   // Internal variables
//   double path_scale_;
//   double path_range_;
//   double robot_diameter_;

//   // last linear velocity applied
//   double last_linear_velocity_;

//   // Carrot
//   Vector3 intermediate_carrot_; // a collision-free intermediate goal before reaching the desired goal
//   Vector3 path_end_; // end of the best chosen path

//   // internal point cloud representations
//   pcl::PointCloud<CollisionPoint>::Ptr collision_map_;
//   pcl::PointCloud<CollisionPoint>::Ptr collision_map_in_base_;
//   pcl::PointCloud<CollisionPoint>::Ptr collision_map_wnormals_in_base_;

//   // Path allocation
//   pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_stack[NUM_MAP_CLOUD_STACK];
//   pcl::PointCloud<pcl::PointXYZ>::Ptr precomputed_paths_[NUM_GROUPS]; // precomputed trajectories

//   pcl::PointCloud<pcl::PointXYZI>::Ptr paths_[NUM_PATHS]; // visualization: all the precomputed trajectories. They are visualized with
//   different rotations pcl::PointCloud<pcl::PointXYZI>::Ptr free_paths_;       // visualization: free paths after collision checking

//   std::vector<int> correspondences_[NUM_GRID_VOXELS]; // the number of correspondences is equal to the number of voxels

//   // Path validity
//   int path_list_[NUM_PATHS] = {0};

//   float end_dir_path_list_[NUM_PATHS] = {0};
//   int clear_path_list_[36 * NUM_PATHS] = {0};

//   float path_likelihood_[36 * NUM_PATHS] = {0}; // The likelihood of each path to reach the goal
//   float path_group_likelihood_[36 * NUM_GROUPS] = {0}; // The likelihood of the group to reach the goal

//   // The quantities below are used to compute the traversability probability of a path
//   float path_traversability_score_[36 * NUM_PATHS] = {0}; // Integrated traversability score along the path
//   int path_traversability_crossed_voxels_[36 * NUM_PATHS] = {0}; // Number of voxels crossed by the path

//   float path_deformation_list_[36 * NUM_PATHS] = {0}; // Deformation of the path due to the terrain

//   // Feasible path found
//   bool feasible_path_found_;
//   int best_path_group_id_;
//   float best_path_group_score_;
//   std::vector<Vector3> best_path_;

//   // // Visualizations
//   // ros::Publisher free_paths_pub_;
//   // ros::Publisher collision_map_pub_;

//   // // Dynamic reconfigure
//   // std::shared_ptr<dynamic_reconfigure::Server<field_local_planner::FalcoControllerConfig>> dynamic_reconfigure_server_;
//   // dynamic_reconfigure::Server<field_local_planner::FalcoControllerConfig>::CallbackType dynamic_reconfigure_callback_;

//   // // Mutex for grid_map manipulation
//   // std::mutex mutex_;
// };
