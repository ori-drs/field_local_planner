#include <locally_reactive_controller/manager.hpp>
#include <locally_reactive_controller/controllers/trackline_controller.hpp>
#include <locally_reactive_controller/controllers/falco_controller.hpp>
#include <locally_reactive_controller/controllers/rmp_controller.hpp>
#include <locally_reactive_controller/utils/utils.hpp>

#include <algorithm>
#include <ros/package.h>

Manager::Manager(const ros::NodeHandle& node, std::string action_name) :
  node_handle_(node),
  action_name_(action_name),
  action_server_(node, action_name, false) {
  
  readParameters();
  setupActionServer();
  setupDynamicReconfigureServer();
  setupServiceServer();

  setupSubscribers();
  setupPublishers();

  initializeVariables();
}

//-------------------------------------------------------------------------------------------------
// Parameters
//-------------------------------------------------------------------------------------------------

void Manager::readParameters() {
  // Initialize controller
  createController( utils::getParameter<std::string>(node_handle_, "controller") );

   // Input topic
  pose_input_topic_      = utils::getParameter<std::string>(node_handle_, "pose_input_topic");
  twist_input_topic_     = utils::getParameterDefault<std::string>(node_handle_, "twist_input_topic", "");
  joy_twist_input_topic_ = utils::getParameterDefault<std::string>(node_handle_, "joy_twist_input_topic", "/cmd_vel");
  elevation_map_topic_   = utils::getParameter<std::string>(node_handle_, "elevation_map_topic");
  // Output type
  locally_reactive_controller_twist_type_ = utils::getParameterDefault<std::string>(node_handle_, "output_type", "twist_stamped");
  // Output topic
  locally_reactive_controller_topic_ = utils::getParameter<std::string>(node_handle_, "output_topic");
  // Output type
  fixed_frame_ = utils::getParameterDefault<std::string>(node_handle_, "fixed_frame", "odom");
  // Base frame
  base_frame_ = utils::getParameterDefault<std::string>(node_handle_, "base_frame", "base");
  // Valid goal frames
  valid_goal_frames_ = utils::getParameterVector<std::string>(node_handle_, "valid_goal_frames");

  // Read control rate
  float maximum_control_command_rate;
  static float micros_in_second = 1000000;
  maximum_control_command_rate = utils::getParameter<float>(node_handle_, "maximum_control_command_rate_hz");
  if (maximum_control_command_rate > 0) {
    ROS_INFO("Applying maximum control signal rate of %f Hz", maximum_control_command_rate);
    publish_interval_microsec_ = (1.0 / maximum_control_command_rate) * micros_in_second;
  } else {
    ROS_INFO("No maximum control signal rate specified, control signals will be published based on the frequency of the pose input topic");
  }
}

void Manager::createController(const std::string& controller_name) {
  // Save controller name
  controller_name_ = controller_name;

  // Read common parameters
  ControllerParameters common_params;
  common_params.name_                             = controller_name_;
  common_params.perceptive_                       = utils::getParameterDefault<bool>(node_handle_, "is_perceptive", false);
  common_params.use_elevation_map_cloud_          = utils::getParameterDefault<bool>(node_handle_, "use_elevation_map_cloud", false);

  common_params.fixed_frame_                      = utils::getParameterDefault<std::string>(node_handle_, "fixed_frame", "odom");
  common_params.base_frame_                       = utils::getParameterDefault<std::string>(node_handle_, "base_frame", "base");
  common_params.robot_length_                     = utils::getParameterDefault<double>(node_handle_, "robot_length", 0.6);
  common_params.robot_width_                      = utils::getParameterDefault<double>(node_handle_, "robot_width", 0.4);
  common_params.robot_height_                     = utils::getParameterDefault<double>(node_handle_, "robot_height", 0.4);
  common_params.robot_clearance_                  = utils::getParameterDefault<double>(node_handle_, "robot_clearance", 0.1);
  common_params.max_forward_linear_velocity_      = utils::getParameter<double>(node_handle_, "max_forward_linear_velocity");
  common_params.max_lateral_linear_velocity_      = utils::getParameter<double>(node_handle_, "max_lateral_linear_velocity");
  common_params.max_angular_velocity_             = utils::getParameter<double>(node_handle_, "max_angular_velocity");
  common_params.max_turning_linear_velocity_      = utils::getParameter<double>(node_handle_, "max_turning_linear_velocity");
  common_params.min_linear_velocity_              = utils::getParameter<double>(node_handle_, "min_linear_velocity");
  common_params.min_angular_velocity_             = utils::getParameter<double>(node_handle_, "min_angular_velocity");
  common_params.goal_distance_threshold_          = utils::getParameter<double>(node_handle_, "goal_distance_threshold");
  common_params.goal_heading_threshold_           = utils::getParameter<double>(node_handle_, "goal_heading_threshold");
  common_params.turn_to_face_heading_threshold_   = utils::getParameter<double>(node_handle_, "turn_to_face_heading_threshold");
  common_params.angular_gain_p_                   = utils::getParameter<double>(node_handle_, "angular_gain_p");
  common_params.linear_gain_p_                    = utils::getParameter<double>(node_handle_, "linear_gain_p");
  common_params.goal_behind_mode_                 = utils::getParameter<int>(node_handle_, "goal_behind_mode");                  // 0 ignore, 1 backwards or 2 turnaround
  common_params.motion_mode_                      = utils::getParameter<int>(node_handle_, "motion_mode");
  common_params.back_is_front_                    = utils::getParameter<bool>(node_handle_, "back_is_front");
  common_params.yaw_exclusive_turns_              = utils::getParameter<bool>(node_handle_, "yaw_exclusive_turns");
  common_params.ignore_intermediate_goal_heading_ = utils::getParameter<bool>(node_handle_, "ignore_intermediate_goal_heading");
  common_params.differential_drive_               = utils::getParameterDefault<bool>(node_handle_, "differential_drive", false);
  common_params.planar_motion_                    = utils::getParameterDefault<bool>(node_handle_, "planar_motion", true);
  common_params.stand_safe_                       = utils::getParameterDefault<bool>(node_handle_, "stand_safe", false);

  common_params.voxel_size_filter_                = utils::getParameterDefault<double>(node_handle_, "voxel_size_filter", 0.05);
  common_params.sensor_range_                     = utils::getParameterDefault<double>(node_handle_, "sensor_range", 3.5);
  common_params.traversable_thr_                  = utils::getParameterDefault<double>(node_handle_, "traversable_thr", 0.7);

  common_params.check_unreachability_             = utils::getParameterDefault<bool>(node_handle_, "check_unreachability", true);
  common_params.unreachability_time_threshold_    = utils::getParameterDefault<double>(node_handle_, "unreachability_time_threshold", 30.0);
  common_params.unreachability_delta_progress_threshold_ = utils::getParameterDefault<double>(node_handle_, "unreachability_delta_progress_threshold", 0.01);

  // Update robot diameter
  common_params.robot_diameter_ = hypot(common_params.robot_length_, common_params.robot_width_);

  // Create controller
  if (controller_name_ == "trackline"){
    controller_ptr_ = std::make_shared<TracklineController>(common_params);

  } else if (controller_name_ == "falco") {
    controller_ptr_ = std::make_shared<FalcoController>(common_params);

  } else if (controller_name_ == "rmp") {
    controller_ptr_ = std::make_shared<RmpController>(common_params);
  
  } else {
    ROS_ERROR_STREAM("Controller [" << controller_name << "] not supported. Exit");
    exit(-1);
  }  
}

//-------------------------------------------------------------------------------------------------
// Setup
//-------------------------------------------------------------------------------------------------
void Manager::setupActionServer() {
  // Create ActionLib Server
  action_server_.registerGoalCallback(boost::bind(&Manager::newGoalRequestActionHandler, this));
  action_server_.registerPreemptCallback(boost::bind(&Manager::preemptActionHandler, this));
  action_server_.start();
}

void Manager::setupDynamicReconfigureServer() {
    // Dynamic Reconfigure
  dynamic_reconfigure_callback_ = boost::bind(&Manager::dynamicReconfigureCallback, this, _1, _2);
  dynamic_reconfigure_server_.setCallback(dynamic_reconfigure_callback_);
}

void Manager::setupServiceServer(){
  pause_service_ = node_handle_.advertiseService("/locally_reactive_controller/pause_execution", &Manager::pauseServiceHandler, this);
  operation_paused_ = false;
}

void Manager::setupSubscribers(){
  // Pose
  pose_sub_ = node_handle_.subscribe(pose_input_topic_, 1, &Manager::poseHandler, this);
  if(twist_input_topic_ != "")
    twist_sub_ = node_handle_.subscribe(twist_input_topic_, 1, &Manager::twistHandler, this);
  
  // Include elevation map
  if(controller_ptr_->getPerceptive()){
    elevation_map_sub_ = node_handle_.subscribe(std::string(elevation_map_topic_), 1, &Manager::elevationMapHandler, this);
  }

  // goal and other subscribers
  driving_goal_sub_    = node_handle_.subscribe(std::string("/driving_plan_request"), 1, &Manager::newDrivingGoalHandler, this);
  driving_rviz_sub_    = node_handle_.subscribe(std::string("/goal"), 1, &Manager::newDrivingGoalRvizHandler, this);
  driving_rviz2_sub_   = node_handle_.subscribe(std::string("/move_base_simple/goal"), 1, &Manager::newDrivingGoalRvizHandler, this); // rviz in certain configurations
  goal_sub_            = node_handle_.subscribe(std::string("/locally_reactive_controller/goal_pose"), 1, &Manager::newGoalRequestHandler, this);
  goal_list_sub_       = node_handle_.subscribe(std::string("/locally_reactive_controller/goal_pose_list"), 1, &Manager::newGoalListRequestHandler, this);
  joy_twist_input_sub_ = node_handle_.subscribe(joy_twist_input_topic_, 1, &Manager::newTwistInputHandler, this);
  stop_sub_            = node_handle_.subscribe(std::string("/locally_reactive_controller/stop_walking_cmd"), 1, &Manager::stopWalkingHandler, this);
  enable_sub_          = node_handle_.subscribe(std::string("/locally_reactive_controller/enable_locally_reactive_controller"), 1, &Manager::enableHandler, this);
}

void Manager::setupPublishers(){
  // Status
  status_pub_ = node_handle_.advertise<locally_reactive_controller_msgs::Status>("/locally_reactive_controller/controller_status", 10);

  // Position controller publication type
  if (locally_reactive_controller_twist_type_ == "twist") {
    velocity_command_pub_ = node_handle_.advertise<geometry_msgs::Twist>(  locally_reactive_controller_topic_, 10);

  } else if (locally_reactive_controller_twist_type_ == "twist_stamped") {
    velocity_command_pub_ = node_handle_.advertise<geometry_msgs::TwistStamped>(  locally_reactive_controller_topic_, 10);

  } else {
    ROS_ERROR("Invalid twist type %s, valid options are 'twist' or 'twist_stamped'", locally_reactive_controller_twist_type_.c_str());
    exit(1);
  }
  ROS_INFO("Publishing commands with type %s", locally_reactive_controller_twist_type_.c_str());

  // Setup visualizer
  visualizer_.setup(node_handle_);
}

void Manager::initializeVariables() {
  previous_status_publish_utime_ = 0;
  previous_visualization_publish_utime_ = 0;
  previous_command_utime_ = 0;
  f_T_fb_tm1_header_.frame_id = "base";
  f_T_fb_tm1_header_.stamp.sec = 0;
  f_T_fb_tm1_header_.stamp.nsec = 0;
  f_T_fb_tm1_ = Eigen::Isometry3d::Identity();
}

//-------------------------------------------------------------------------------------------------
// ActionLib
//-------------------------------------------------------------------------------------------------
void Manager::newGoalRequestActionHandler(){
  ROS_INFO_STREAM("LocallyReactiveController: New goal received - actionlib");
  geometry_msgs::PoseStamped goal = action_server_.acceptNewGoal()->goal;
  geometry_msgs::PoseStampedConstPtr goal_pointer( new geometry_msgs::PoseStamped(goal) );
  setNewGoal(goal_pointer);
}

void Manager::preemptActionHandler(){
  ROS_INFO_STREAM("LocallyReactiveController: preemptActionHandler - stop walking");
  action_server_.setPreempted();
  std_msgs::Int16ConstPtr stop_pointer( new std_msgs::Int16() );
  // stopWalkingHandler(stop_pointer);

  publishZeroVelocityCommand();
}

//-------------------------------------------------------------------------------------------------
// Dynamic Reconfigure
//-------------------------------------------------------------------------------------------------
void Manager::dynamicReconfigureCallback(locally_reactive_controller::LocallyReactiveControllerConfig &config, uint32_t level){
    // Controller parameters
  ControllerParameters common_params = controller_ptr_->getParameters();
  common_params.fixed_frame_ = fixed_frame_;
  common_params.base_frame_  = base_frame_;
  common_params.name_ = controller_name_;
  utils::assignAndPrintDiff("robot_length", common_params.robot_length_, config.robot_length);
  utils::assignAndPrintDiff("robot_width", common_params.robot_width_, config.robot_width);
  utils::assignAndPrintDiff("robot_height", common_params.robot_height_, config.robot_height);
  utils::assignAndPrintDiff("robot_clearance", common_params.robot_clearance_, config.robot_clearance);
  utils::assignAndPrintDiff("max_forward_velocity", common_params.max_forward_linear_velocity_, config.max_forward_linear_velocity);
  utils::assignAndPrintDiff("max_lateral_linear_velocity", common_params.max_lateral_linear_velocity_, config.max_lateral_linear_velocity);
  utils::assignAndPrintDiff("max_angular_velocity", common_params.max_angular_velocity_, config.max_angular_velocity);
  utils::assignAndPrintDiff("max_turning_linear_velocity", common_params.max_turning_linear_velocity_, config.max_turning_linear_velocity);
  utils::assignAndPrintDiff("min_linear_velocity", common_params.min_linear_velocity_, config.min_linear_velocity);
  utils::assignAndPrintDiff("min_angular_velocity", common_params.min_angular_velocity_, config.min_angular_velocity);
  utils::assignAndPrintDiff("goal_distance_threshold", common_params.goal_distance_threshold_, config.goal_distance_threshold);
  utils::assignAndPrintDiff("goal_heading_threshold", common_params.goal_heading_threshold_, config.goal_heading_threshold);
  utils::assignAndPrintDiff("turn_to_face_heading_threshold", common_params.turn_to_face_heading_threshold_, config.turn_to_face_heading_threshold);
  utils::assignAndPrintDiff("angular_gain_p", common_params.angular_gain_p_, config.angular_gain_p);
  utils::assignAndPrintDiff("linear_gain_p", common_params.linear_gain_p_, config.linear_gain_p);
  utils::assignAndPrintDiff("goal_behind_mode", common_params.goal_behind_mode_, config.goal_behind_mode);
  utils::assignAndPrintDiff("motion_mode", common_params.motion_mode_, config.motion_mode);
  utils::assignAndPrintDiff("back_is_front", common_params.back_is_front_, config.back_is_front);
  utils::assignAndPrintDiff("yaw_exclusive_turns", common_params.yaw_exclusive_turns_, config.yaw_exclusive_turns);
  utils::assignAndPrintDiff("ignore_intermediate_goal_heading", common_params.ignore_intermediate_goal_heading_, config.ignore_intermediate_goal_heading);
  utils::assignAndPrintDiff("differential_drive", common_params.differential_drive_, config.differential_drive);
  utils::assignAndPrintDiff("planar_motion", common_params.planar_motion_, config.planar_motion);
  utils::assignAndPrintDiff("stand_safe", common_params.stand_safe_, config.stand_safe);
  utils::assignAndPrintDiff("voxel_size_filter", common_params.voxel_size_filter_, config.voxel_size_filter);
  utils::assignAndPrintDiff("sensor_range", common_params.sensor_range_, config.sensor_range);
  utils::assignAndPrintDiff("traversable_thr", common_params.traversable_thr_, config.traversable_thr);

  utils::assignAndPrintDiff("check_unreachability", common_params.check_unreachability_, config.check_unreachability);
  utils::assignAndPrintDiff("unreachability_time_threshold", common_params.unreachability_time_threshold_, config.unreachability_time_threshold);
  utils::assignAndPrintDiff("unreachability_delta_progress_threshold", common_params.unreachability_delta_progress_threshold_, config.unreachability_delta_progress_threshold);

  // Update robot diameter
  common_params.robot_diameter_ = hypot(common_params.robot_length_, common_params.robot_width_);

  // Update common params
  controller_ptr_->setParameters(common_params);

  ROS_INFO("Updated common parameters with dynamic_reconfigure");
}

//-------------------------------------------------------------------------------------------------
// Services
//-------------------------------------------------------------------------------------------------
bool Manager::pauseServiceHandler(std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res)
{
  std::string message = "LocallyReactiveController pausing operation";
  if (!req.data)
    message = "LocallyReactiveController unpausing operation";
    
  ROS_INFO_STREAM(message);

  res.success = true;
  res.message = message;
  operation_paused_ = req.data;

  if (operation_paused_) { 
    // I think this doesn't work - the message never makes it out. perhaps there service handler blocks publishing?
    ROS_INFO_STREAM("Sending zero velocity command to pause robot");
    publishZeroVelocityCommand();
    controller_ptr_->stopWalking();
  }

  return true;
}

//-------------------------------------------------------------------------------------------------
// Subscription callbacks
//-------------------------------------------------------------------------------------------------
void Manager::poseHandler(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  // Save data
  Eigen::Isometry3d f_T_fb_t;
  std::string f_T_fb_t_frame;
  utils::poseWithCovarianceStampedToIsometry3dAndFrame(msg, f_T_fb_t, f_T_fb_t_frame);

  // ROS_INFO("---------- Control step ---------------------------------------------"); 
  // Ignore if not within the control rate
  int64_t msg_utime = utils::getHeaderUTime(msg->header);
  if ((msg_utime - previous_command_utime_) < publish_interval_microsec_){
    f_T_fb_tm1_ = f_T_fb_t;
    f_T_fb_tm1_header_ = msg->header;
    return;
  }
    
  // Check if operation is paused
  if (operation_paused_){
    ROS_INFO_THROTTLE(1,"LocallyReactiveController is paused. Not outputting any commands");
    return;
  }

  // Compute control command
  ControllerBase::ControllerStatus controller_status = controller_ptr_->compute(f_T_fb_t, msg->header);
  Eigen::Isometry3d f_Tgoal_f_g = controller_ptr_->getCurrentGoal();
  // Publish status and goal as tf
  publishControllerStatus(msg_utime, controller_status);

  Eigen::Vector3d linear_velocity;
  Eigen::Vector3d angular_velocity;

  // Publish control command
  switch(controller_status.action_) {
    ROS_DEBUG("Action: SEND_NOTHING");
    case ControllerBase::OutputAction::SEND_NOTHING:
      break;

    case ControllerBase::OutputAction::SEND_STOP_WALKING:
      ROS_DEBUG("Action: SEND_STOP_WALKING");
      publishZeroVelocityCommand();
      break;

    case ControllerBase::OutputAction::SEND_COMMAND:
      ROS_DEBUG("Action: SEND_COMMAND");
      linear_velocity = controller_ptr_->getLinearVelocity();
      angular_velocity = controller_ptr_->getAngularVelocity();
      publishVelocityCommand(linear_velocity, angular_velocity);
      break;
    
    case ControllerBase::OutputAction::SEND_UNREACHABLE:
      ROS_DEBUG("Action: SEND_STOP_WALKING: Unreachable goal");
      publishZeroVelocityCommand();
      break;
  }
  
  // Visualize
  publishVisualizations();
  publishGoalAsTf(msg->header.stamp, f_Tgoal_f_g);

  // Update internal variables
  f_T_fb_tm1_ = f_T_fb_t;
  f_T_fb_tm1_header_ = msg->header;
  previous_command_utime_ = msg_utime;
}

void Manager::twistHandler(const geometry_msgs::TwistWithCovarianceStampedConstPtr& msg) {
  Vector6d velocity = Vector6d::Zero();
  velocity(0) = msg->twist.twist.angular.x;
  velocity(1) = msg->twist.twist.angular.y;
  velocity(2) = msg->twist.twist.angular.z;
  velocity(3) = msg->twist.twist.linear.x;
  velocity(4) = msg->twist.twist.linear.y;
  velocity(5) = msg->twist.twist.linear.z;
  
  controller_ptr_->setVelocity(velocity);
}

void Manager::elevationMapHandler(const grid_map_msgs::GridMap& grid_map_msg){
  // ROS_INFO("[Manager] New elevation map message");

  grid_map::GridMap grid_map;
  // Convert grid map  
  grid_map::GridMapRosConverter::fromMessage(grid_map_msg, grid_map);
  grid_map.convertToDefaultStartIndex();

  // Set to controller
  controller_ptr_->setElevationMap(grid_map);
}

void Manager::newDrivingGoalHandler(const geometry_msgs::PoseStampedConstPtr& msg){
  ROS_INFO_STREAM("New driving goal received");
  setNewGoal(msg);
}

void Manager::newTwistInputHandler(const geometry_msgs::TwistConstPtr& msg){
  ROS_INFO_STREAM("New velocity input received");
  
  // Extract components
  Eigen::Vector3d linear_velocity;
  linear_velocity << msg->linear.x, msg->linear.y, msg->linear.z;
  Eigen::Vector3d angular_velocity;
  angular_velocity << msg->angular.x, msg->angular.y, msg->angular.z;

  // Note: Here we integrate position and then rotation to make a compound transformation
  // The ideal formulation would be to use the SE(3) exponential map

  // Integrate
  double dt = 1.0;
  Eigen::Isometry3d dT = Eigen::Isometry3d::Identity();

  // Integrate translation
  Eigen::Vector3d d_position = linear_velocity * dt;
  dT.translate(d_position);

  // Integrate rotation
  Eigen::Vector3d d_angle = angular_velocity * dt;
  double angle = d_angle.norm();
  Eigen::Vector3d axis = d_angle.normalized();
  Eigen::AngleAxisd angle_axis(angle, axis);
  dT.rotate(angle_axis);

  // Add delta to current pose
  dT = f_T_fb_tm1_ * dT;

  // Create and assign new pose msg
  geometry_msgs::PoseStampedPtr pose_msg = boost::make_shared<geometry_msgs::PoseStamped>(); // Hack to use the same usual goal type
  pose_msg->header.frame_id = f_T_fb_tm1_header_.frame_id;
  tf::poseEigenToMsg(dT, pose_msg->pose);
  
  // Set new artificial goal
  setNewGoal(pose_msg);
}


void Manager::newDrivingGoalRvizHandler(const geometry_msgs::PoseStampedConstPtr& msg){
  ROS_INFO_STREAM("New driving goal received");
  setNewGoal(msg);
}

void Manager::newGoalRequestHandler(const geometry_msgs::PoseStampedConstPtr& msg){
  ROS_INFO_STREAM("New driving goal received");
  setNewGoal(msg);
}

void Manager::setNewGoal(const geometry_msgs::PoseStampedConstPtr& msg){
  ROS_INFO_STREAM("New goal with frame_id: " << msg->header.frame_id );

  // Check if the new goal has a valid frame
  if (std::find(valid_goal_frames_.begin(), valid_goal_frames_.end(), msg->header.frame_id) == valid_goal_frames_.end()) {
    ROS_WARN_THROTTLE(1, "Goal frame '%s' is not in the list of valid goal frames. Rejected. This message is throttled (1 s)", msg->header.frame_id.c_str());
    action_server_.setAborted();
    return;
  }

  // Convert goal message
  Eigen::Isometry3d goal_pose;
  std::string goal_pose_frame;
  utils::poseStampedToIsometry3dAndFrame(msg, goal_pose, goal_pose_frame);

  // Convert to fixed frame
  ROS_INFO("Automatically enabling position controller");
  controller_ptr_->setGoal(goal_pose, goal_pose_frame, f_T_fb_tm1_);
}

void Manager::newGoalListRequestHandler(const geometry_msgs::PoseArrayConstPtr& msg){
  
  ROS_INFO_STREAM("GOAL_REQUEST goal list received");
  std::deque<Eigen::Isometry3d> goals;
  for (size_t i =0 ; i < msg->poses.size() ; i ++){
    Eigen::Isometry3d goal_pose;
    tf::poseMsgToEigen(msg->poses[i], goal_pose);
    goals.push_back(goal_pose);
  }

  // Get goal frame
  std::string goal_pose_frame = msg->header.frame_id;

  // Fill controller
   ROS_INFO("Automatically enabling position controller to a list of goals");
  controller_ptr_->setGoalList(goals, goal_pose_frame, f_T_fb_tm1_);
 
  //positionController_->setGoalListAndEnable( goals, previousPose_ );
}

void Manager::stopWalkingHandler(const std_msgs::Int16ConstPtr& msg){
  ROS_INFO("STOP_WALKING received. Following disabled and command zero velocity");

  publishZeroVelocityCommand();
  controller_ptr_->stopWalking();
}

void Manager::enableHandler(const std_msgs::StringConstPtr& msg){
  std::cout << "ENABLE_PATH_FOLLOWER received. Not re-implemented yet within ROS launch/cfg\n";
  // This loads a trajectory and executes the path
}

//-------------------------------------------------------------------------------------------------
// Publishers
//-------------------------------------------------------------------------------------------------
void Manager::publishVelocityCommand(const Eigen::Vector3d& linear_velocity, const Eigen::Vector3d& angular_velocity){
  geometry_msgs::TwistStamped cmd;
  cmd.header = f_T_fb_tm1_header_;
  cmd.twist.linear.x  = linear_velocity(0);
  cmd.twist.linear.y  = linear_velocity(1);
  cmd.twist.linear.z  = linear_velocity(2);
  cmd.twist.angular.x = angular_velocity(0);
  cmd.twist.angular.y = angular_velocity(1);
  cmd.twist.angular.z = angular_velocity(2);

  if (locally_reactive_controller_twist_type_ == "twist") {
    velocity_command_pub_.publish(cmd.twist);
  } else {
    velocity_command_pub_.publish(cmd);
  }
}

void Manager::publishZeroVelocityCommand(){
  Eigen::Vector3d zero_linear_velocity  = Eigen::Vector3d::Zero();
  Eigen::Vector3d zero_angular_velocity = Eigen::Vector3d::Zero();
  publishVelocityCommand(zero_linear_velocity, zero_angular_velocity);
}

void Manager::publishControllerStatus(int64_t msg_utime, ControllerBase::ControllerStatus controller_status){
  // only publish at control rate
  if ((msg_utime - previous_status_publish_utime_) < publish_interval_microsec_)
    return;
  previous_status_publish_utime_ = msg_utime;

  // Create message
  locally_reactive_controller_msgs::Status msg;
  msg.header = f_T_fb_tm1_header_;
  msg.progress = controller_status.progress_;
  msg.goal_reached = controller_status.goal_reached_;

  // Set status
  switch(controller_status.action_) {
    case ControllerBase::OutputAction::SEND_NOTHING:
      msg.status = locally_reactive_controller_msgs::Status::STOPPED;
      break;
    case ControllerBase::OutputAction::SEND_STOP_WALKING:
      msg.status = locally_reactive_controller_msgs::Status::STOPPED;
      break;
    case ControllerBase::OutputAction::SEND_COMMAND:
      msg.status = locally_reactive_controller_msgs::Status::RUNNING;
      break;
    case ControllerBase::OutputAction::SEND_UNREACHABLE:
      msg.status = locally_reactive_controller_msgs::Status::UNREACHABLE;
      break;
    default:
      msg.status = locally_reactive_controller_msgs::Status::STOPPED;
      break;
  }
  
  // publish status
  status_pub_.publish(msg);
  
  // Sent to the action client
  if (action_server_.isActive()){
    feedback_.distance_to_goal =  controller_ptr_->getDistanceToGoal();
    action_server_.publishFeedback(feedback_);
  }
}


void Manager::publishGoalAsTf(ros::Time stamp, const Eigen::Isometry3d& goal){
  // Publish tf with goal
  tf::Transform goal_transform;
  tf::transformEigenToTF(goal, goal_transform);
  goal_transform.setRotation(goal_transform.getRotation().normalize());
  tf_broadcaster_.sendTransform(tf::StampedTransform(goal_transform, stamp, fixed_frame_, "locally_reactive_controller_goal"));
}

void Manager::publishVisualizations() {
  
  // Visualizations in fixed frame
  f_T_fb_tm1_header_.frame_id = fixed_frame_;
  visualizer_.publishCurrentGoal(controller_ptr_->getCurrentGoal(), f_T_fb_tm1_header_);
  visualizer_.publishRemainingGoals(controller_ptr_->getGoals(), f_T_fb_tm1_header_);
  visualizer_.publishStartingGoal(controller_ptr_->getStartingPose(), f_T_fb_tm1_header_);

  // All the visualizations below are in base frame
  std_msgs::Header base_header = f_T_fb_tm1_header_;
  base_header.frame_id = base_frame_;
  
  // Publish velocity command
  visualizer_.publishEstimatedVelocity(controller_ptr_->getCurrentEstimatedLinearVelocity(), controller_ptr_->getCurrentEstimatedAngularVelocity(), base_header);
  visualizer_.publishVelocityCombined(controller_ptr_->getLinearVelocity(), controller_ptr_->getAngularVelocity(), base_header);
  visualizer_.publishReferencePath(controller_ptr_->getPathToGoal(), base_header);
  visualizer_.publishCollisionBox(controller_ptr_->getParams().robot_length_,
                                  controller_ptr_->getParams().robot_width_,
                                  controller_ptr_->getParams().robot_height_,
                                  controller_ptr_->getParams().robot_clearance_,
                                  controller_ptr_->isColliding(),
                                  base_header);
}

