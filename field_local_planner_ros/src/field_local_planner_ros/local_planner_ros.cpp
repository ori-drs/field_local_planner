#include <field_local_planner_ros/local_planner_ros.hpp>

namespace field_local_planner {

LocalPlannerRos::LocalPlannerRos(ros::NodeHandle& nh) : action_server_(nh, "action_server", false) {
  // Load parameters from parameter server
  loadParameters(nh);

  // Load local planner plugin
  loadPlugin(nh);

  // Setup ROS
  setupRos(nh);

  // Set spinner
  ros::spin();
}

void LocalPlannerRos::loadParameters(ros::NodeHandle& nh) {
  local_planner_name_ = utils::getParameter<std::string>(nh, "local_planner");
}

void LocalPlannerRos::loadPlugin(ros::NodeHandle& nh) {
  // Prepare loader
  plugin_loader_ = std::make_shared<pluginlib::ClassLoader<field_local_planner::BasePlugin>>("field_local_planner_base_plugin",
                                                                                             "field_local_planner::BasePlugin");

  // Prepare plugin name to be loaded
  std::stringstream planner_plugin;  // It should be like "field_local_planner::FalcoPlugin"
  planner_plugin << local_planner_name_;

  try {
    // Create instance
    local_planner_plugin_ = plugin_loader_->createInstance(planner_plugin.str());

    // Load parameters
    local_planner_plugin_->initialize(nh);

  } catch (pluginlib::PluginlibException& ex) {
    ROS_FATAL_STREAM("The plugin [" << planner_plugin.str() << "] failed to load for some reason. Error: " << ex.what());
    exit(-1);
  }
}

void LocalPlannerRos::setupRos(ros::NodeHandle& nh) {
  // Setup subscribers
  std::string pose_topic = utils::getParameterDefault(nh, "pose_topic", std::string(""));
  if (!pose_topic.empty()) {
    pose_sub_ = nh.subscribe(pose_topic, 1, &LocalPlannerRos::poseCallback, this);
  }

  std::string twist_topic = utils::getParameterDefault(nh, "twist_topic", std::string(""));
  if (!twist_topic.empty()) {
    twist_sub_ = nh.subscribe(twist_topic, 1, &LocalPlannerRos::twistCallback, this);
  }

  std::string odometry_topic = utils::getParameterDefault(nh, "odometry_topic", std::string(""));
  if (!odometry_topic.empty()) {
    odometry_sub_ = nh.subscribe(odometry_topic, 1, &LocalPlannerRos::odometryCallback, this);
  }

  // Sensing
  // std::string rgb_image_topic = utils::getParameterDefault(nh, "rgb_image_topic", std::string(""));
  // if (!rgb_image_topic.empty()) {
  //   // TODO add synchronized subscriber
  //   rgb_image_sub_ = nh.subscribe(std::string(rgb_image_topic), 1, &LocalPlannerRos::imageRgbCallback, this);
  // }

  // std::string rgbd_image_topic = utils::getParameterDefault(nh, "rgbd_image_topic", std::string(""));
  // if (!rgbd_image_topic.empty()) {
  //   // TODO add synchronized subscriber
  //   rgbd_image_sub_ = nh.subscribe(std::string(rgbd_image_topic), 1, &LocalPlannerRos::imageRgbdCallback, this);
  // }

  // std::string depth_image_topic = utils::getParameterDefault(nh, "depth_image_topic", std::string(""));
  // if (!depth_image_topic.empty()) {
  //   // TODO add synchronized subscriber
  //   depth_image_sub_ = nh.subscribe(std::string(depth_image_topic), 1, &LocalPlannerRos::imageDepthCallback, this);
  // }

  std::string point_cloud_topic = utils::getParameterDefault(nh, "point_cloud_topic", std::string(""));
  if (!point_cloud_topic.empty()) {
    point_cloud_sub_ = nh.subscribe(std::string(point_cloud_topic), 1, &LocalPlannerRos::pointCloudCallback, this);
  }

  std::string grid_map_topic = utils::getParameterDefault(nh, "grid_map_topic", std::string(""));
  if (!grid_map_topic.empty()) {
    grid_map_sub_ = nh.subscribe(std::string(grid_map_topic), 1, &LocalPlannerRos::gridMapCallback, this);
  }

  // Goal
  std::string goal_topic = utils::getParameterDefault(nh, "goal_topic", std::string("/goal"));
  goal_sub_ = nh.subscribe(std::string(goal_topic), 1, &LocalPlannerRos::goalCallback, this);

  // Joystick command
  std::string joy_twist_topic = utils::getParameterDefault(nh, "joy_twist_topic", std::string("/goal"));
  joy_twist_sub_ = nh.subscribe(std::string(joy_twist_topic), 1, &LocalPlannerRos::joyTwistCallback, this);

  // Setup publishers
  // Status
  status_pub_ = nh.advertise<field_local_planner_msgs::Status>("/field_local_planner/status", 10);
  // Path
  path_pub_ = nh.advertise<nav_msgs::Path>("/field_local_planner/path", 10);

  // Output twist type type
  output_twist_type_ = utils::getParameterDefault(nh, "output_twist_type", std::string("twist"));
  std::string output_twist_topic = utils::getParameterDefault(nh, "output_twist_topic", std::string("/field_local_planner/twist"));
  if (output_twist_type_ == "twist") {
    output_twist_pub_ = nh.advertise<geometry_msgs::Twist>(output_twist_topic, 10);

  } else if (output_twist_type_ == "twist_stamped") {
    output_twist_pub_ = nh.advertise<geometry_msgs::TwistStamped>(output_twist_topic, 10);

  } else {
    ROS_ERROR_STREAM("Invalid twist type [" << output_twist_type_ << "], valid options are 'twist' or 'twist_stamped'");
    exit(1);
  }

  // Setup services

  // Setup action server
  action_server_.registerGoalCallback(boost::bind(&LocalPlannerRos::newGoalRequestActionHandler, this));
  action_server_.registerPreemptCallback(boost::bind(&LocalPlannerRos::preemptActionHandler, this));
  action_server_.start();
}

void LocalPlannerRos::poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg) {
  local_planner_plugin_->setPose(pose_msg->pose.pose, pose_msg->header);

  // Execute local planner
  geometry_msgs::Twist twist;
  nav_msgs::Path path;
  field_local_planner_msgs::Status status;
  bool valid = local_planner_plugin_->execute(pose_msg->header.stamp, twist, path, status);

  if(valid) {
    publishTwist(twist);
    publishPath(path);
    publishStatus(status);
  }

}

void LocalPlannerRos::twistCallback(const geometry_msgs::TwistWithCovarianceStampedConstPtr& twist_msg) {
  local_planner_plugin_->setVelocity(twist_msg->twist.twist, twist_msg->header);
}

void LocalPlannerRos::odometryCallback(const nav_msgs::OdometryConstPtr& odo_msg) {
  local_planner_plugin_->setPose(odo_msg->pose.pose, odo_msg->header);
  local_planner_plugin_->setVelocity(odo_msg->twist.twist, odo_msg->header);
}

// // Sensor callbacks
// void LocalPlannerRos::imageRgbCallback(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::CameraInfoConstPtr& info_msg) {
//   local_planner_plugin_->setImageRgb(rgb_msg, info_msg);
// }

// void LocalPlannerRos::imageRgbdCallback(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::ImageConstPtr& depth_msg,
//                                             const sensor_msgs::CameraInfoConstPtr& info_msg) {
//   local_planner_plugin_->setImageRgbd(rgb_msg, depth_msg, info_msg);
// }

// void LocalPlannerRos::imageDepthCallback(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::CameraInfoConstPtr&
// info_msg) {
//   local_planner_plugin_->setImageDepth(depth_msg, info_msg);
// }

void LocalPlannerRos::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
  local_planner_plugin_->setPointCloud(cloud_msg);
}

void LocalPlannerRos::gridMapCallback(const grid_map_msgs::GridMap& gridmap_msg) {
  local_planner_plugin_->setGridMap(gridmap_msg);
}

// Goal callback
void LocalPlannerRos::goalCallback(const geometry_msgs::PoseStampedConstPtr& goal_msg) {
  local_planner_plugin_->setGoal(goal_msg->pose, goal_msg->header);
}

// Joystick callback
void LocalPlannerRos::joyTwistCallback(const geometry_msgs::TwistConstPtr& twist_msg) {
  local_planner_plugin_->setJoyCommand(*twist_msg);
}

// Action server
void LocalPlannerRos::newGoalRequestActionHandler() {
  ROS_INFO_STREAM("[LocalPlannerRos] Action Server - New goal");

  // Get goal from server
  geometry_msgs::PoseStamped goal_msg = action_server_.acceptNewGoal()->goal;

  // Pass goal to the local planner
  local_planner_plugin_->setGoal(goal_msg.pose, goal_msg.header);
}

void LocalPlannerRos::preemptActionHandler() {
  ROS_INFO_STREAM("[LocalPlannerRos] Action Server - Stop");
  action_server_.setPreempted();

  publishZeroTwist();
}

void LocalPlannerRos::publishTwist(const geometry_msgs::Twist& twist) {
  if (output_twist_type_ == "twist") {
    output_twist_pub_.publish(twist);

  } else if (output_twist_type_ == "twist_stamped") {
    geometry_msgs::TwistStamped twist_stamped;
    twist_stamped.twist = twist;
    twist_stamped.header.seq = 0;
    twist_stamped.header.stamp = ros::Time::now();  // Check if this is correct
    twist_stamped.header.frame_id = local_planner_plugin_->getBaseFrame();
    output_twist_pub_.publish(twist_stamped);
  }
}

void LocalPlannerRos::publishZeroTwist() {
  geometry_msgs::Twist zero_twist;
  zero_twist.angular.x = 0.0;
  zero_twist.angular.y = 0.0;
  zero_twist.angular.z = 0.0;
  zero_twist.linear.x = 0.0;
  zero_twist.linear.y = 0.0;
  zero_twist.linear.z = 0.0;
  publishTwist(zero_twist);
}

void LocalPlannerRos::publishPath(const nav_msgs::Path& path) {
  path_pub_.publish(path);
}

void LocalPlannerRos::publishStatus(const field_local_planner_msgs::Status& status) {
  status_pub_.publish(status);
}

}  // namespace field_local_planner
