#pragma once
#include <string>
#include <deque>
#include <Eigen/Dense>
#include <grid_map_core/GridMap.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

#include <grid_map_ros/GridMapRosConverter.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>

#include <ros/ros.h>
#include <std_msgs/Header.h>

#include <locally_reactive_controller/utils/utils.hpp>
#include <locally_reactive_controller/utils/profiler.hpp>
#include <locally_reactive_controller/utils/timer.hpp>
#include <locally_reactive_controller_msgs/Status.h>

// This struct defines the basic parameters required by a ControllerBase
class ControllerParameters {
public:
  std::string fixed_frame_; // Fixed frame for control
  std::string base_frame_; // Fixed frame for control

  double robot_length_ = 0.7;
  double robot_width_  = 0.5;
  double robot_height_ = 0.5;
  double robot_clearance_ = 0.1; // to define a safety area around the robot
  double robot_diameter_ = hypot(robot_length_, robot_width_);
  
  double max_forward_linear_velocity_;
  double max_lateral_linear_velocity_;
  double max_angular_velocity_;
  double max_turning_linear_velocity_;    // when turning, dont use the same max velocities
  double min_linear_velocity_;
  double min_angular_velocity_;
  double goal_distance_threshold_;
  double goal_heading_threshold_;
  double turn_to_face_heading_threshold_;
  // Proportional controller gains
  double angular_gain_p_;
  double linear_gain_p_;
  // modes
  int goal_behind_mode_;                  // 0 ignore, 1 backwards or 2 turnaround
  int motion_mode_;                       // 0 turn/walk/turn or 1 shuffle
  bool back_is_front_;                    // true to consider the back of the robot as the front
  bool yaw_exclusive_turns_;              // true to apply only yaw when turning
  bool ignore_intermediate_goal_heading_; // don't rotate to align to the goal heading if it is not a single goal or the last goal in a list
  bool differential_drive_;               // Ignores lateral velocities
  bool planar_motion_ = true;             // Ignores roll, pitch and z components
  bool stand_safe_ = false;               // Computes a control command even when the robot is just standing

  bool perceptive_ = false;               // If the ControllerBase uses perception data
  bool use_elevation_map_cloud_ = false;  // If the ControllerBase requires to convert the elevation map to a cloud
  
  std::string name_ = "default_controller"; // Name of the ControllerBase

  // Perceptive stuff
  double voxel_size_filter_ = 0.05;        // voxel grid filter
  double sensor_range_ = 3.5;              // sensor range
  double traversable_thr_ = 0.9;           // traversability threshold

  // Unreachability detection
  bool check_unreachability_ = true;
  double unreachability_delta_progress_threshold_ = 0.01;
  double unreachability_time_threshold_ = 30.0; // seconds
};

class ControllerBase {
  // Definitions
  using Vector6d = Eigen::Matrix<double, 6, 1>;

public:
  enum MotionMode{
    TURN_WALK_TURN = 0,
    SHUFFLE = 1,
  };

  enum class State {
    UNKNOWN = -1,            // UNKNOWN only occurs when starting
    FINISHED = 0,            // FINISHED robot at goal. Do nothing
    TURN_TO_DESTINATION = 1, // TURN_TO_DESTINATION robot at goal but with wrong heading. turn in place to desired heading
    FORWARD = 2,             // FORWARD robot far from goal but facing it. walk forward
    TURN_TO_GOAL = 3,        // TURN_TO_GOAL robot far from/reached the goal and not facing it. turn in place to reach the desired orientation
    GOAL_BEHIND = 4,         // GOAL_BEHIND goal is behind the robot. Do nothing unless flag is activated
    UNREACHABLE=5,             // The goal is close but the robot cannot reach it for some reason
  };

  enum OutputAction {
    SEND_NOTHING,       // Do nothing
    SEND_STOP_WALKING,  // Sends command to stop walking
    SEND_COMMAND,       // Sends computed control comand
    SEND_UNREACHABLE,   // Cannot reach the goal
  };

  struct ControllerStatus {
    OutputAction action_;
    double progress_;
    bool goal_reached_;
  };
  
  // Constructor
  // ControllerBase();
  ControllerBase(const ControllerParameters& params);

  // General methods for any ControllerBase
  void setVelocity(const Vector6d& base_velocity);
  // Set goal
  void setGoal(const Eigen::Isometry3d& goal, std::string goal_frame, const Eigen::Isometry3d& current_pose);
  // Set list of goals
  void setGoalList(const std::deque<Eigen::Isometry3d>& goals, std::string goal_frame, const Eigen::Isometry3d& current_pose);
  // Stop walking
  void stopWalking();
  // Set parameters
  void setParameters(const ControllerParameters& params);
  
  // Get methods
  ControllerParameters& getParameters() { return params_; }
  
  bool getNextGoal(Eigen::Isometry3d current_pose);

  Eigen::Vector3d getCurrentEstimatedLinearVelocity() const { return b_v_b_t_.tail<3>(); }
  Eigen::Vector3d getCurrentEstimatedAngularVelocity() const { return b_v_b_t_.head<3>(); }
  Eigen::Vector3d getLinearVelocity() const { return output_linear_velocity_; }
  Eigen::Vector3d getAngularVelocity() const { return output_angular_velocity_; }
  double getDistanceToGoal() const { return error_distance_to_goal_; }
  double getOrientationToGoal() const { return error_orientation_to_goal_; }
  double getProgress() const { return progress_t_; }

  std::deque<Eigen::Isometry3d> getGoals() const { return goals_; }
  size_t getNumGoals() const { return goals_.size(); }
  Eigen::Isometry3d getCurrentGoal() const { return f_Tgoal_fb_t_; }
  Eigen::Isometry3d getStartingPose() const { return f_Tstart_fb_t_; }
  std::vector<Eigen::Vector3d> getPathToGoal() { return path_to_goal_; }
  const ControllerParameters& getParams() const { return params_; }
  bool isColliding() const { return is_colliding_; }

  // Return basic properties of the controller
  bool getPerceptive() const { return params_.perceptive_; }
  std::string getName() const  { return params_.name_; }

  // Compute control command. Main method
  ControllerStatus compute(const Eigen::Isometry3d& current_pose, const std_msgs::Header& header);

  // Check and convert goal frame
  bool convertGoalToFixedFrame();
  // Check if there are valid goals queued to be followed
  bool checkValidGoals();
  // Check if back is front is enabled and flip the robot pose
  void checkBackIsFront();
  // Computes the distance and orientation error to goal
  void computeDistanceAndOrientationToGoal();
  // Check if we cannot reach the goal
  bool checkUnreachableState();
  // Check ControllerBase state
  void checkControllerState();
  // Check if there are new goals in the queue
  void checkNewGoal();
  // Compute control command
  void computeControlCommand();
  // Enforce velocity limits to output velocities
  void enforceVelocityLimits();
  // Enforce velocity limits for a single component
  void enforceVelocityLimits(double& velocity, const double& min_velocity, const double& max_velocity);

  // Perceptive methods
  void setElevationMap(const grid_map::GridMap& grid_map);

  // Methods to be implemented by each ControllerBase
  virtual void updateControllerParameters(ros::NodeHandle& node_handle) = 0;
  virtual void setupVisualizations(ros::NodeHandle& node_handle) = 0;
  virtual void publishVisualizations(const std_msgs::Header& header) = 0;
  virtual void setupDynamicReconfigureServer(ros::NodeHandle& node_handle) = 0;

private:
  virtual void customPreProcessLocalMap() = 0;
  virtual void customPreProcessController() = 0;
  virtual OutputAction computeCommandGoalBehind(Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d& output_angular_velocity) = 0;
  virtual OutputAction computeCommandTurnToGoal(Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d& output_angular_velocity) = 0;
  virtual OutputAction computeCommandForward(Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d& output_angular_velocity) = 0;
  virtual OutputAction computeCommandTurnToDestination(Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d& output_angular_velocity) = 0;
  virtual OutputAction computeCommandFinished(Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d& output_angular_velocity) = 0;
  virtual void computeCommandPostProcess(ControllerBase::OutputAction& action, 
                                          Eigen::Vector3d& output_linear_velocity,
                                          Eigen::Vector3d& output_angular_velocity,
                                          std::vector<Eigen::Vector3d>& path_to_goal) = 0;

public:
  //TF listener
  tf::TransformListener tf_listener_;

  // Node handle
  ros::NodeHandle node_handle_;

  // Params
  ControllerParameters params_;

  // ControllerBase state
  State state_;

  // Controller output
  ControllerStatus controller_status_;

  // Controller rate
  double controller_dt_;

  // Frames
  std::string goals_frame_;
  std::string fixed_frame_;

  // List of goals
  std::deque<Eigen::Isometry3d> goals_;
  // Current goal
  Eigen::Isometry3d g_Tgoal_gb_t_; // Goal in original goal frame
  Eigen::Isometry3d f_Tgoal_fb_t_; // Goal in fixed control frame

  // Pose of robot (in fixed frame) when it started moving to goal pose.
  Eigen::Isometry3d f_Tstart_fb_t_;

  // Current pose
  Eigen::Isometry3d f_T_fb_t_;
  Eigen::Isometry2d f_T_fb_t_2d_;
  double ts_t_;
  // Previous pose
  Eigen::Isometry3d f_T_fb_tm1_;
  double ts_tm1_;

  // Delta pose to goal
  Eigen::Isometry3d dT_base_goal_;
  // Robot velocity
  bool use_external_velocity_;
  Vector6d b_v_b_t_;

  // Distance to goal
  double error_distance_to_goal_;
  double error_heading_to_goal_;
  double error_orientation_to_goal_;
  double error_orientation_to_starting_pose_;
  double distance_start_to_goal_;

  // Progress
  double progress_t_;
  double progress_tm1_;
  
  // Some flags to control specific behaviours
  bool has_got_near_to_current_goal_;
  bool can_rotate_;
  bool is_colliding_;
  bool goal_closer_to_back_;
  bool potential_unreachability_;

  // If the controller reached the current goal
  bool finished_with_this_goal_;
  bool initial_rotation_executed_;

  // Perceptive stuff
  // Transformations used to properly handle the elevation map messages
  bool new_elevation_map_;
  Eigen::Isometry3d b_T_bm_tmap_; // map frame to base frame
  Eigen::Isometry3d f_T_fb_tmap_; // base to fixed frame

  // Internal grid map
  grid_map::GridMap grid_map_;

  // Profiler
  std::shared_ptr<Profiler> profiler_ptr_;

protected:
  // Computed velocities
  Eigen::Vector3d output_linear_velocity_;
  Eigen::Vector3d output_angular_velocity_;
  
  // Path to goal
  std::vector<Eigen::Vector3d> path_to_goal_;

  // Timer to identify unreachable state
  // Timer unreachability_timer_;
  double unreachability_ts_;
};
