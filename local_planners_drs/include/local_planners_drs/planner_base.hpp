#pragma once

#include <local_planners_drs/profiler.hpp>
#include <local_planners_drs/timer.hpp>

#include <grid_map_core/GridMap.hpp>

#include <opencv2/core/core.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <yaml-cpp/yaml.h>
#include <string>


namespace local_planners_drs {

using namespace gtsam;

using Twist = gtsam::Vector6;

class Base {

public:
  struct Parameters
  {
    double robot_length = 1.0;
    double robot_width = 1.0;
    double robot_height = 1.0;
    bool requires_sensing = false;
    double distance_to_goal_thr = 0.1;    // meters
    double orientation_to_goal_thr = 0.1; // radians
    double max_linear_velocity_x = 1.0;   // m/s
    double max_linear_velocity_y = 1.0;   // m/s
    double max_angular_velocity_z = 1.0;  // rad/s
  };

  // Possible local planner states
  enum class Status
  {
    NOT_READY = 0,  // Initial state
    FINISHED = 1,   // Robot at goal. Do nothing
    EXECUTING = 2,  // Robot trying to reach the goal
    FAILURE = 5,    // Robot cannot make any progress, potentially a failure 
  };

  // This defines the output of the local planner
  struct Output {
    Twist twist ;
    Status status;
  };
  
public:
  // Compute the twist command - this function depends on the local planner
  virtual Twist computeTwist() = 0;

  // Check failure - it may depend on the method
  bool checkFailure();
  
protected:
  // Constructor
  Base();

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
  void setRobotState(const Pose3& T_f_b, const Twist& b_v);

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
  Pose3 T_b_s_pc_;    // Pose of point cloud in base frame
  Pose3 T_b_s_gm_;    // Pose of grid map in base frame

  // Robot state
  Pose3 T_f_b_; // Pose of body in fixed frame 
  Twist b_v_;   // Twist in base frame
  
  // Helper transforms
  Pose3 T_m_f_; // Pose of fixed frame in map frame

  // Goal
  Pose3 T_f_b_start_; // Pose when the goal was set
  Pose3 T_f_g_; // Goal in fixed frame
  
  // Helpers
  Pose3 dT_b_g_;
  Pose3 dT_b_start_;
  double distance_to_goal_;
  double orientation_to_goal_;
  double distance_to_start_;
  double orientation_to_start_;
  double heading_towards_goal_; // How much the robot should rotate to point towards the goal

  // Other flags
  bool sensing_ready_;

//   // General methods for any LocalPlannerBase
//   void setVelocity(const Vector6d& base_velocity);
//   // Set goal
//   void setGoal(const Eigen::Isometry3d& goal, std::string goal_frame, const Eigen::Isometry3d& current_pose);
//   // Set list of goals
//   void setGoalList(const std::deque<Eigen::Isometry3d>& goals, std::string goal_frame, const Eigen::Isometry3d& current_pose);
//   // Stop walking
//   void stopWalking();
//   // Set parameters
//   void setParameters(const ControllerParameters& params);
  
//   // Get methods
//   ControllerParameters& getParameters() { return params_; }
  
//   bool getNextGoal(Eigen::Isometry3d current_pose);

//   Eigen::Vector3d getCurrentEstimatedLinearVelocity() const { return b_v_b_t_.tail<3>(); }
//   Eigen::Vector3d getCurrentEstimatedAngularVelocity() const { return b_v_b_t_.head<3>(); }
//   Eigen::Vector3d getLinearVelocity() const { return output_linear_velocity_; }
//   Eigen::Vector3d getAngularVelocity() const { return output_angular_velocity_; }
//   double getDistanceToGoal() const { return error_distance_to_goal_; }
//   double getOrientationToGoal() const { return error_orientation_to_goal_; }
//   double getProgress() const { return progress_t_; }

//   std::deque<Eigen::Isometry3d> getGoals() const { return goals_; }
//   size_t getNumGoals() const { return goals_.size(); }
//   Eigen::Isometry3d getCurrentGoal() const { return f_Tgoal_fb_t_; }
//   Eigen::Isometry3d getStartingPose() const { return f_Tstart_fb_t_; }
//   std::vector<Eigen::Vector3d> getPathToGoal() { return path_to_goal_; }
//   const ControllerParameters& getParams() const { return params_; }
//   bool isColliding() const { return is_colliding_; }

//   // Return basic properties of the controller
//   bool getPerceptive() const { return params_.perceptive_; }
//   std::string getName() const  { return params_.name_; }

//   // Compute control command. Main method
//   ControllerStatus compute(const Eigen::Isometry3d& current_pose, const std_msgs::Header& header);

//   // Check and convert goal frame
//   bool convertGoalToFixedFrame();
//   // Check if there are valid goals queued to be followed
//   bool checkValidGoals();
//   // Check if back is front is enabled and flip the robot pose
//   void checkBackIsFront();
//   // Computes the distance and orientation error to goal
//   void computeDistanceAndOrientationToGoal();
//   // Check if we cannot reach the goal
//   bool checkUnreachableState();
//   // Check LocalPlannerBase state
//   void checkControllerState();
//   // Check if there are new goals in the queue
//   void checkNewGoal();
//   // Compute control command
//   void computeControlCommand();
//   // Enforce velocity limits to output velocities
//   void enforceVelocityLimits();
//   // Enforce velocity limits for a single component
//   void enforceVelocityLimits(double& velocity, const double& min_velocity, const double& max_velocity);

//   // Perceptive methods
//   void setElevationMap(const grid_map::GridMap& grid_map);

//   // Methods to be implemented by each LocalPlannerBase
//   virtual void updateControllerParameters(ros::NodeHandle& node_handle) = 0;
//   virtual void setupVisualizations(ros::NodeHandle& node_handle) = 0;
//   virtual void publishVisualizations(const std_msgs::Header& header) = 0;
//   virtual void setupDynamicReconfigureServer(ros::NodeHandle& node_handle) = 0;

// private:
//   virtual void customPreProcessLocalMap() = 0;
//   virtual void customPreProcessController() = 0;
//   virtual OutputAction computeCommandGoalBehind(Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d& output_angular_velocity) = 0;
//   virtual OutputAction computeCommandTurnToGoal(Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d& output_angular_velocity) = 0;
//   virtual OutputAction computeCommandForward(Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d& output_angular_velocity) = 0;
//   virtual OutputAction computeCommandTurnToDestination(Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d& output_angular_velocity) = 0;
//   virtual OutputAction computeCommandFinished(Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d& output_angular_velocity) = 0;
//   virtual void computeCommandPostProcess(LocalPlannerBase::OutputAction& action, 
//                                           Eigen::Vector3d& output_linear_velocity,
//                                           Eigen::Vector3d& output_angular_velocity,
//                                           std::vector<Eigen::Vector3d>& path_to_goal) = 0;

// public:
//   //TF listener
//   tf::TransformListener tf_listener_;

//   // Node handle
//   ros::NodeHandle node_handle_;

//   // Params
//   ControllerParameters params_;

//   // LocalPlannerBase state
//   State state_;

//   // Controller output
//   ControllerStatus controller_status_;

//   // Controller rate
//   double controller_dt_;

//   // Frames
//   std::string goals_frame_;
//   std::string fixed_frame_;

//   // List of goals
//   std::deque<Eigen::Isometry3d> goals_;
//   // Current goal
//   Eigen::Isometry3d g_Tgoal_gb_t_; // Goal in original goal frame
//   Eigen::Isometry3d f_Tgoal_fb_t_; // Goal in fixed control frame

//   // Pose of robot (in fixed frame) when it started moving to goal pose.
//   Eigen::Isometry3d f_Tstart_fb_t_;

//   // Current pose
//   Eigen::Isometry3d f_T_fb_t_;
//   Eigen::Isometry2d f_T_fb_t_2d_;
//   double ts_t_;
//   // Previous pose
//   Eigen::Isometry3d f_T_fb_tm1_;
//   double ts_tm1_;

//   // Delta pose to goal
//   Eigen::Isometry3d dT_base_goal_;
//   // Robot velocity
//   bool use_external_velocity_;
//   Vector6d b_v_b_t_;

//   // Distance to goal
//   double error_distance_to_goal_;
//   double error_heading_to_goal_;
//   double error_orientation_to_goal_;
//   double error_orientation_to_starting_pose_;
//   double distance_start_to_goal_;

//   // Progress
//   double progress_t_;
//   double progress_tm1_;
  
//   // Some flags to control specific behaviours
//   bool has_got_near_to_current_goal_;
//   bool can_rotate_;
//   bool is_colliding_;
//   bool goal_closer_to_back_;
//   bool potential_unreachability_;

//   // If the controller reached the current goal
//   bool finished_with_this_goal_;
//   bool initial_rotation_executed_;

//   // Perceptive stuff
//   // Transformations used to properly handle the elevation map messages
//   bool new_elevation_map_;
//   Eigen::Isometry3d b_T_bm_tmap_; // map frame to base frame
//   Eigen::Isometry3d f_T_fb_tmap_; // base to fixed frame

//   // Internal grid map
//   grid_map::GridMap grid_map_;

//   // Profiler
//   std::shared_ptr<Profiler> profiler_ptr_;

// protected:
//   // Computed velocities
//   Eigen::Vector3d output_linear_velocity_;
//   Eigen::Vector3d output_angular_velocity_;
  
//   // Path to goal
//   std::vector<Eigen::Vector3d> path_to_goal_;

//   // Timer to identify unreachable state
//   // Timer unreachability_timer_;
//   double unreachability_ts_;
};

} // namespace local_planners_drs