#pragma once
#include <locally_reactive_controller/controllers/controller_base.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <dynamic_reconfigure/server.h>
#include <locally_reactive_controller/RmpControllerConfig.h>

#include <visualization_msgs/MarkerArray.h>

// RMP stuff
#include <gtsam/base/Vector.h>
#include <rmp/rmp.hpp>

#include <map>

class RmpControllerParameters {
public:

  RmpControllerParameters() {}

  std::string config_folder_ = "/controllers/rmp/"; // pathFolder

  double sphere_radius_factor_ = 1.0;
  double geodesic_goal_gain_ = 7.0;
  double geodesic_goal_offset_ = 0.1;
  double geodesic_goal_steepness_ = 1.0;

  double geodesic_heading_gain_ = 7.0;
  double geodesic_heading_offset_ = 0.1;
  double geodesic_heading_steepness_ = 1.0;

  double goal_position_gain_ = 2.0;
  double goal_position_offset_ = 0.1;
  double goal_position_steepness_ = 1.0;

  double goal_heading_gain_ = 2.0;
  double goal_heading_offset_ = 0.1;
  double goal_heading_steepness_ = 1.0;

  double damping_ = 1.0;
  double obstacle_gain_ = 1.0;
  double obstacle_offset_ = 0.1;
  double obstacle_steepness_ = 1.0;
  double heading_gain_ = 1.0;
  double heading_offset_ = 1.0;
  double heading_steepness_ = 1.0;
  double regularization_ = 0.1;
  double integration_time_ = 1.0;
  // weights
  double geodesic_goal_weight_ = 1.0;
  double geodesic_heading_weight_ = 1.0;
  double goal_position_weight_ = 1.0;
  double goal_heading_weight_ = 1.0;
  double obstacle_weight_ = 1.0;
  double damping_weight_ = 1.0;
  double heading_weight_ = 1.0;
  bool check_internal_collisions_ = false; 
};

class RmpController : public ControllerBase {

  struct ControlPointStruct {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // ID
    std::string id_;
    // Point coordinates
    Eigen::Vector2d point_;
    // Point coordinates in fixed frame
    Eigen::Vector2d point_fixed_;
    // Color
    Eigen::Vector3f color_;
    // Accelerations
    Eigen::Vector2d acc_;
    // Metric
    Eigen::Matrix2d metric_;
    // Affected by forces
    bool affected_by_goal_;
    bool affected_by_obstacle_;
    // Radius for spheres
    float radius_;

    ControlPointStruct() = default;
  };

  struct AccelerationVisualization {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // ID
    std::string id_;
    // Point coordinates
    Eigen::Vector2d point_;
    // Colors
    Eigen::Vector3f color_;
    // Accelerations
    Eigen::Vector2d acc_;
    double angular_acc_;
    // Metric
    Eigen::Matrix2d metric_;
    // weight
    double weight_;
  };


public:
  RmpController(const ControllerParameters& params);
  
  // Set parameters
  void updateControllerParameters(ros::NodeHandle& node_handle);

  // Visualizations
  void setupVisualizations(ros::NodeHandle& node_handle);
  void publishVisualizations(const std_msgs::Header& header);

  // Dynamic reconfigure
  void setupDynamicReconfigureServer(ros::NodeHandle& node_handle);
  void dynamicReconfigureCallback(locally_reactive_controller::RmpControllerConfig &config, uint32_t level);

private:
  // Controller implementation
  void customPreProcessLocalMap();
  void customPreProcessController();
  ControllerBase::OutputAction computeCommandGoalBehind(Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d& output_angular_velocity);
  ControllerBase::OutputAction computeCommandTurnToGoal(Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d& output_angular_velocity);
  ControllerBase::OutputAction computeCommandForward(Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d& output_angular_velocity);
  ControllerBase::OutputAction computeCommandTurnToDestination(Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d& output_angular_velocity);
  ControllerBase::OutputAction computeCommandFinished(Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d& output_angular_velocity);
  void computeCommandPostProcess(ControllerBase::OutputAction& action, 
                                 Eigen::Vector3d& output_linear_velocity,
                                 Eigen::Vector3d& output_angular_velocity,
                                 std::vector<Eigen::Vector3d>& path_to_goal);

  // RMP computation
  void computeOptimalAcceleration();
  void computeOptimalAccelerationDifferential();
  void computeOptimalAccelerationCombined();

  // Preallocates the control points
  void prepareControlPoints();
 
  // Utils
  // void resetControlPointsValidity();
  visualization_msgs::MarkerArray controlPointsToMarkerArray(const std::vector<ControlPointStruct>& control_points,
                                                             const std_msgs::Header& header);
  visualization_msgs::MarkerArray accelerationsToMarkerArray(const std::vector<AccelerationVisualization>& accs,
                                                             const std_msgs::Header& header);

  // Variables
  RmpControllerParameters controller_params_;

  // Internal
  gtsam::Vector3 velocity_2d_;
  gtsam::Vector3 optimal_velocity_;
  gtsam::Vector3 optimal_acc_;
  gtsam::Matrix3 optimal_metric_;
  Eigen::Isometry3d current_pose_in_map_;
  gtsam::Pose2 gtsam_current_pose_in_map_;
  gtsam::Pose2 gtsam_current_pose_in_fixed_;
  gtsam::Pose2 gtsam_current_goal_in_fixed_;

  // Control points stuff
  std::vector<ControlPointStruct> control_points_;
  std::vector<AccelerationVisualization> vis_accelerations_;
  
  // std::vector<gtsam::ControlPoint2> control_points_;
  // std::vector<gtsam::ControlPoint2_> control_points_expressions_;
  // std::vector<bool> valid_control_points_;
  // std::vector<bool> goal_affected_control_points_;
  // std::vector<bool> obstacle_affected_control_points_;
  std::map<std::string, size_t> control_points_indices_;
  
  // Visualizations
  // std::vector<ControlPoint2Vis> vis_control_points_;
  // std::vector<ControlPoint2Vis> vis_control_points_goal_;
  ros::Publisher control_points_pub_;
  ros::Publisher accelerations_pub_;

  // Dynamic reconfigure
  std::shared_ptr<dynamic_reconfigure::Server<locally_reactive_controller::RmpControllerConfig>> dynamic_reconfigure_server_;
  dynamic_reconfigure::Server<locally_reactive_controller::RmpControllerConfig>::CallbackType dynamic_reconfigure_callback_;

  // Mutex for grid_map manipulation
  std::mutex mutex_;
};