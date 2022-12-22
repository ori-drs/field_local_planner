#include <field_local_planners_ros/local_planner_handler.hpp>

namespace field_local_planners {

LocalPlannerHandler::LocalPlannerHandler(ros::NodeHandle& nh) : action_server_(nh, "field_local_planners_ros", false) {
  // Load parameters from parameter server
  loadParameters(nh);

  // Load local planner plugin
  loadPlugin(nh);

  // Setup ROS
  setupRos(nh);
}

void LocalPlannerHandler::loadParameters(ros::NodeHandle& nh) {
  local_planner_name_ = utils::getParameter<std::string>(nh, "local_planner");
  fixed_frame_ = utils::getParameter<std::string>(nh, "fixed_frame");
  base_frame_  = utils::getParameter<std::string>(nh, "base_frame");
}

void LocalPlannerHandler::loadPlugin(ros::NodeHandle& nh) {
  // Prepare loader
  pluginlib::ClassLoader<field_local_planners::BasePlugin> plugin_loader("field_local_planners_plugins", "field_local_planners::BasePlugin");

  try {
    // Prepare plugin name to be loaded
    std::stringstream planner_plugin;  // It should be like "field_local_planners::FalcoPlugin"
    // Create instance
    local_planner_ = plugin_loader.createInstance(planner_plugin.str());

    // Load parameters
    local_planner_->initialize(nh);
  } catch (pluginlib::PluginlibException& ex) {
    ROS_FATAL("The plugin failed to load for some reason. Error: %s", ex.what());
    exit(-1);
  }
}

void LocalPlannerHandler::setupRos(ros::NodeHandle& nh) {
  // Setup subscribers
  std::string pose_topic = utils::getParameterDefault(nh, "pose_topic", std::string(""));
  if (!pose_topic.empty()) {
    pose_sub_ = nh.subscribe(pose_topic, 1, &LocalPlannerHandler::poseCallback, this);
  }

  std::string twist_topic = utils::getParameterDefault(nh, "twist_topic", std::string(""));
  if (!twist_topic.empty()) {
    twist_sub_ = nh.subscribe(twist_topic, 1, &LocalPlannerHandler::twistCallback, this);
  }

  std::string odometry_topic = utils::getParameterDefault(nh, "odometry_topic", std::string(""));
  if (!odometry_topic.empty()) {
    odometry_sub_ = nh.subscribe(odometry_topic, 1, &LocalPlannerHandler::odometryCallback, this);
  }

  // Sensing
  std::string rgb_image_topic = utils::getParameterDefault(nh, "rgb_image_topic", std::string(""));
  if (!rgb_image_topic.empty()) {
    rgb_image_sub_ = nh.subscribe(std::string(rgb_image_topic), 1, &LocalPlannerHandler::gridMapCallback, this);
  }

  std::string rgbd_image_topic = utils::getParameterDefault(nh, "rgbd_image_topic", std::string(""));
  if (!rgbd_image_topic.empty()) {
    rgbd_image_sub_ = nh.subscribe(std::string(rgbd_image_topic), 1, &LocalPlannerHandler::gridMapCallback, this);
  }

  std::string depth_image_topic = utils::getParameterDefault(nh, "depth_image_topic", std::string(""));
  if (!depth_image_topic.empty()) {
    depth_image_sub_ = nh.subscribe(std::string(depth_image_topic), 1, &LocalPlannerHandler::gridMapCallback, this);
  }

  std::string point_cloud_topic = utils::getParameterDefault(nh, "point_cloud_topic", std::string(""));
  if (!point_cloud_topic.empty()) {
    point_cloud_sub_ = nh.subscribe(std::string(point_cloud_topic), 1, &LocalPlannerHandler::gridMapCallback, this);
  }

  std::string grid_map_topic = utils::getParameterDefault(nh, "grid_map_topic", std::string(""));
  if (!grid_map_topic.empty()) {
    grid_map_sub_ = nh.subscribe(std::string(grid_map_topic), 1, &LocalPlannerHandler::gridMapCallback, this);
  }

  // Goal
  std::string goal_topic = utils::getParameterDefault(nh, "depth_image_topic", std::string("/goal"));
  goal_sub_ = nh.subscribe(std::string(goal_topic), 1, &LocalPlannerHandler::goalCallback, this);

  // Joystick command
  std::string joy_twist_topic = utils::getParameterDefault(nh, "joy_twist_topic", std::string("/goal"));
  joy_twist_sub_ = nh.subscribe(std::string(joy_twist_topic), 1, &LocalPlannerHandler::joyTwistCallback, this);

  // Setup publishers
  // Status
  status_pub_ = nh.advertise<field_local_planners_msgs::Status>("/field_local_planners/status", 10);

  // Output twist type type
  std::string output_twist_type = utils::getParameterDefault(nh, "output_twist_type", std::string("twist"));
  std::string output_twist_topic = utils::getParameterDefault(nh, "output_twist_topic", std::string("/field_local_planners/twist"));
  if (output_twist_type == "twist") {
    output_twist_pub_ = nh.advertise<geometry_msgs::Twist>(output_twist_topic, 10);

  } else if (output_twist_type == "twist_stamped") {
    output_twist_pub_ = nh.advertise<geometry_msgs::TwistStamped>(output_twist_topic, 10);

  } else {
    ROS_ERROR_STREAM("Invalid twist type " << output_twist_type << ", valid options are 'twist' or 'twist_stamped'");
    exit(1);
  }

  // Setup services

  // Setup action server
  action_server_.registerGoalCallback(boost::bind(&LocalPlannerHandler::newGoalRequestActionHandler, this));
  action_server_.registerPreemptCallback(boost::bind(&LocalPlannerHandler::preemptActionHandler, this));
  action_server_.start();
}

void LocalPlannerHandler::poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
  //
  Pose3 T_f_b;

  local_planner_->setPoseInFixed(T_f_b);
}
void LocalPlannerHandler::twistCallback(const geometry_msgs::TwistWithCovarianceStampedConstPtr& msg) {
  Twist b_v = Twist::Zero();
  // We use GTSAM's convention with orientation-then-position (i.e, angular velocity and then linear)
  b_v(0) = msg->twist.twist.angular.x;
  b_v(1) = msg->twist.twist.angular.y;
  b_v(2) = msg->twist.twist.angular.z;
  b_v(3) = msg->twist.twist.linear.x;
  b_v(4) = msg->twist.twist.linear.y;
  b_v(5) = msg->twist.twist.linear.z;
  local_planner_->setVelocityInBase(b_v);
}
void LocalPlannerHandler::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  Pose3 T_f_b;
  Twist b_v = Twist::Zero();
  //

  local_planner_->setPoseInFixed(T_f_b);
  local_planner_->setVelocityInBase(b_v);
}

// Sensor callbacks
void LocalPlannerHandler::imageRgbCallback(const sensor_msgs::ImageConstPtr& rgb_msg) {
  //
}

void LocalPlannerHandler::imageRgbdCallback(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::ImageConstPtr& depth_msg) {
  //
}

void LocalPlannerHandler::imageDepthCallback(const sensor_msgs::ImageConstPtr& depth_msg) {
  //
}

void LocalPlannerHandler::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
  //
}

void LocalPlannerHandler::gridMapCallback(const grid_map_msgs::GridMap& cloud_msg) {
  //
}

// Goal callback
void LocalPlannerHandler::goalCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  // Convert goal msg to Pose3


  

  // Update goal tf
  tf::Transform goal_transform;
  tf::transformEigenToTF(goal, goal_transform);
  goal_transform.setRotation(goal_transform.getRotation().normalize());
  tf_broadcaster_.sendTransform(tf::StampedTransform(goal_transform, ros::Time::now(), fixed_frame_, "local_planner_goal"));
}

// Joystick callback
void LocalPlannerHandler::joyTwistCallback(const geometry_msgs::TwistConstPtr& msg) {
  //
}

// Action server
void LocalPlannerHandler::newGoalRequestActionHandler() {
  ROS_INFO_STREAM("[LocalPlannerHandler] Action Server - New goal");

  // Get goal from server
  geometry_msgs::PoseStamped goal = action_server_.acceptNewGoal()->goal;

  // Convert message to current frame

  // Pass goal to the local planner
  local_planner_->setGoalInFixed();
}

void LocalPlannerHandler::preemptActionHandler() {
  ROS_INFO_STREAM("[LocalPlannerHandler] Action Server - Stop");
  action_server_.setPreempted();

  publishZeroTwist();
}

void LocalPlannerHandler::publishTwist(const Twist& twist) {}

void LocalPlannerHandler::publishZeroTwist() {
  publishTwist(Twist::Zero());
}

}  // namespace field_local_planners
