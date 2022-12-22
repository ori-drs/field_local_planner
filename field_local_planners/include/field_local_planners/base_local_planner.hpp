#pragma once

#include <field_local_planners/profiler.hpp>
#include <field_local_planners/timer.hpp>

#include <grid_map_core/GridMap.hpp>

#include <opencv2/core/core.hpp>

#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <yaml-cpp/yaml.h>
#include <string>

namespace field_local_planners {

using namespace gtsam;

using Twist = gtsam::Vector6;

class BaseLocalPlanner {
 public:
  struct Parameters {
    double robot_length = 1.0;
    double robot_width = 1.0;
    double robot_height = 1.0;
    bool requires_sensing = false;
    double distance_to_goal_thr = 0.1;     // meters
    double orientation_to_goal_thr = 0.1;  // radians
    double max_linear_velocity_x = 1.0;    // m/s
    double max_linear_velocity_y = 1.0;    // m/s
    double max_angular_velocity_z = 1.0;   // rad/s
  };

  // Possible local planner states
  enum class Status {
    NOT_READY = 0,  // Initial state
    FINISHED = 1,   // Robot at goal. Do nothing
    EXECUTING = 2,  // Robot trying to reach the goal
    FAILURE = 5,    // Robot cannot make any progress, potentially a failure
  };

  // This defines the output of the local planner
  struct Output {
    Twist twist;
    Status status;
  };

 public:
  // Compute the twist command - this function depends on the local planner
  virtual Twist computeTwist() = 0;

  // Check failure - it may depend on the method
  bool checkFailure();

 public:
  // Constructor
  BaseLocalPlanner();

  // Initializer
  void initialize(const Parameters& parameters);

  // Main method to execute the local planner
  Output execute();

  // Other steps
  Status checkStatus();

  void computeDistanceAndOrientationToGoal();

  // Interfaces for external data
  void setImageRgb(const cv::Mat& img, const Pose3& T_b_s);
  void setImageRgbd(const cv::Mat& img, const Pose3& T_b_s);
  void setImageDepth(const cv::Mat& img, const Pose3& T_b_s);
  void setPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const Pose3& T_b_s);
  void setGridMap(const grid_map::GridMap& grid_map, const Pose3& T_b_s);

  // Set state (pose + twist)
  void setPoseInFixed(const Pose3& T_f_b);
  void setVelocityInBase(const Twist& b_v);

  // Set the T_m_f transform to compute stuff in the fixed frame f into the map frame m
  void setFixedToMapTransform(const Pose3& T_m_f);

  // Set single goal
  void setGoalInFixed(const Pose3& T_f_g, const Pose3& T_f_b);

 protected:
  // Parameters
  Parameters parameters_;

  // Possible sensor modalities and their sensor poses in the base frame
  cv::Mat image_rgb_;
  cv::Mat image_rgbd_;
  cv::Mat image_depth_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_;
  grid_map::GridMap grid_map_;
  Pose3 T_b_s_rgb_;
  Pose3 T_b_s_rgbd_;
  Pose3 T_b_s_depth_;
  Pose3 T_b_s_pc_;  // Pose of point cloud in base frame
  Pose3 T_b_s_gm_;  // Pose of grid map in base frame

  // Robot state
  Pose3 T_f_b_;  // Pose of body in fixed frame
  Twist b_v_;    // Twist in base frame

  // Helper transforms
  Pose3 T_m_f_;  // Pose of fixed frame in map frame

  // Goal
  Pose3 T_f_b_start_;  // Pose when the goal was set
  Pose3 T_f_g_;        // Goal in fixed frame

  // Helpers
  Pose3 dT_b_g_;
  Pose3 dT_b_start_;
  double distance_to_goal_;
  double orientation_to_goal_;
  double distance_to_start_;
  double orientation_to_start_;
  double heading_towards_goal_;  // How much the robot should rotate to point towards the goal

  // Other flags
  bool sensing_ready_;
};

}  // namespace field_local_planners