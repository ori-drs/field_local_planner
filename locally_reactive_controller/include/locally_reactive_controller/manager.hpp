#pragma once

#include <locally_reactive_controller/visualizer.hpp>
#include <locally_reactive_controller/controllers/controller_base.hpp>

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

#include <grid_map_msgs/GridMap.h>
#include <grid_map_core/GridMap.hpp>
#include <grid_map_ros/grid_map_ros.hpp>

#include <dynamic_reconfigure/server.h>
#include <locally_reactive_controller/LocallyReactiveControllerConfig.h>

#include <actionlib/server/simple_action_server.h>
#include <locally_reactive_controller/LocallyReactiveControllerAction.h>

#include <locally_reactive_controller_msgs/Status.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <memory>

class Manager{
  using Vector6d = Eigen::Matrix<double, 6, 1>;

public:
  // Constructor
  Manager(const ros::NodeHandle& node, std::string action_name);
  
  // Read ROS parameters and prepare subscribers/publishers
  void setup();

private:
  // Setup
  void readParameters();
  void setupActionServer();
  void setupDynamicReconfigureServer();
  void setupServiceServer();

  void setupSubscribers();
  void setupPublishers();
  void initializeVariables();

  void createController(const std::string& controller_name);

  // Callbacks
  // ActionLib Handlers
  void newGoalRequestActionHandler();
  void preemptActionHandler();

  // Callback to dynamic reconfigure
  void dynamicReconfigureCallback(locally_reactive_controller::LocallyReactiveControllerConfig &config, uint32_t level);

  // Service Handlers
  bool pauseServiceHandler(std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res);

  // Main Message handlers (pose and elevation map)
  void poseHandler(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
  void twistHandler(const geometry_msgs::TwistWithCovarianceStampedConstPtr& msg);
  void elevationMapHandler(const grid_map_msgs::GridMap&  grid_map_msg);

  // Goal Handlers
  void newDrivingGoalHandler(const geometry_msgs::PoseStampedConstPtr& msg);
  void newDrivingGoalRvizHandler(const geometry_msgs::PoseStampedConstPtr& msg);
  void newGoalRequestHandler(const geometry_msgs::PoseStampedConstPtr& msg);
  void newTwistInputHandler(const geometry_msgs::TwistConstPtr& msg);
  void setNewGoal(const geometry_msgs::PoseStampedConstPtr& msg);
  void newGoalListRequestHandler(const geometry_msgs::PoseArrayConstPtr& msg);

  // Interface Handlers
  void stopWalkingHandler(const std_msgs::Int16ConstPtr& msg);
  void enableHandler(const std_msgs::StringConstPtr& msg);

  // Publishers
  void publishVelocityCommand(const Eigen::Vector3d& linear_velocity, const Eigen::Vector3d& angular_velocity);
  void publishZeroVelocityCommand();
  void publishControllerStatus(int64_t msg_utime, ControllerBase::ControllerStatus controller_status);
  void publishGoalAsTf(ros::Time stamp, const Eigen::Isometry3d& goal);
  void publishVisualizations();

  // Utils
  bool convertGoalToFixedFrame(Eigen::Isometry3d& goal, std::string frame);

  // Internal variables
  //TF listener
  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;

  // Helper objects
  Visualizer visualizer_;
  std::shared_ptr<ControllerBase> controller_ptr_;

  // ROS Stuff
  ros::NodeHandle node_handle_;

  // Subscribers
  ros::Subscriber pose_sub_;
  ros::Subscriber twist_sub_;
  ros::Subscriber elevation_map_sub_;
  
  ros::Subscriber driving_goal_sub_;
  ros::Subscriber driving_rviz_sub_;
  ros::Subscriber driving_rviz2_sub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber goal_list_sub_;
  ros::Subscriber joy_twist_input_sub_;

  ros::Subscriber stop_sub_;
  ros::Subscriber enable_sub_;

  // Publishers
  ros::Publisher velocity_command_pub_;
  ros::Publisher status_pub_;

  // Services
  ros::ServiceServer pause_service_;

  // ActionLib
  actionlib::SimpleActionServer<locally_reactive_controller::LocallyReactiveControllerAction> action_server_;
  locally_reactive_controller::LocallyReactiveControllerResult result_;
  locally_reactive_controller::LocallyReactiveControllerFeedback feedback_;
  std::string action_name_;

  // Dynamic reconfigure
  dynamic_reconfigure::Server<locally_reactive_controller::LocallyReactiveControllerConfig> dynamic_reconfigure_server_;
  dynamic_reconfigure::Server<locally_reactive_controller::LocallyReactiveControllerConfig>::CallbackType dynamic_reconfigure_callback_;

  // Internal
  bool operation_paused_;
  float publish_interval_microsec_;
  std::string pose_input_topic_;
  std::string twist_input_topic_;
  std::string joy_twist_input_topic_;
  std::string elevation_map_topic_;
  std::string locally_reactive_controller_twist_type_;
  std::string locally_reactive_controller_topic_;
  std::string fixed_frame_; // fixed frame used for control
  std::string base_frame_; // used for visualizations

  std::string controller_name_;

  // Pose
  Eigen::Isometry3d f_T_fb_tm1_; // pose of base (b) frame expressed in fixed (f) frame, in time t-1 (tm1)
  std_msgs::Header f_T_fb_tm1_header_;

  // Valid Frames
  std::vector<std::string> valid_goal_frames_;

  // Times
  int64_t previous_status_publish_utime_;
  int64_t previous_visualization_publish_utime_;
  int64_t previous_command_utime_;
};