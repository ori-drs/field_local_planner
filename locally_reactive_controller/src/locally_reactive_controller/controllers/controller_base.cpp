#include <locally_reactive_controller/controllers/controller_base.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <ros/console.h>

// ControllerBase::ControllerBase() :
//       new_elevation_map_(false) {

//   g_Tgoal_gb_t_.translation()[0] = std::numeric_limits<double>::infinity();
//   has_got_near_to_current_goal_ = false;
//   can_rotate_ = true;
//   is_colliding_ = false;
//   goal_closer_to_back_ = false;
//   initial_rotation_executed_ = false;
//   profiler_ptr_ = std::make_shared<Profiler>("ControllerBase");
// };

ControllerBase::ControllerBase(const ControllerParameters& params) :
    params_(params) {
  
  g_Tgoal_gb_t_.translation()[0] = std::numeric_limits<double>::infinity();
  has_got_near_to_current_goal_ = false;
  can_rotate_ = true;
  is_colliding_ = false;
  goal_closer_to_back_ = false;
  profiler_ptr_ = std::make_shared<Profiler>("ControllerBase");
  use_external_velocity_ = false;

  initial_rotation_executed_ = false;
}

void ControllerBase::setParameters(const ControllerParameters& params){
  params_ = params;
}

// Compute control command. Main method
ControllerBase::ControllerStatus ControllerBase::compute(const Eigen::Isometry3d& current_pose, const std_msgs::Header& header) {
  ROS_INFO_STREAM_THROTTLE(1, "----------compute control ----------");
  // Save previous pose and timestamp
  f_T_fb_tm1_ = f_T_fb_t_;
  ts_tm1_ = ts_t_;
  // Save current pose and timestamp
  f_T_fb_t_ = current_pose;
  ts_t_ = header.stamp.toSec();;
 
  profiler_ptr_->startEvent("0.controller_compute");

  // If the controller is perceptive execute the stuff below
  if(params_.perceptive_){
    // Elevation map stuff (if applies)
    if(!grid_map_.exists("elevation")){
      ROS_WARN_THROTTLE(1.0, "Elevation map is not valid/hasn't arrived yet. Controller will SEND_NOTHING. Output is throttled (1 s)");
      // TODO check what happens here
    }
    profiler_ptr_->startEvent("0.1.custom_preprocess_local_map");
    customPreProcessLocalMap();
    profiler_ptr_->endEvent("0.1.custom_preprocess_local_map");
  }

  // Check if there are valid goals
  if(!checkValidGoals()){
    profiler_ptr_->endEvent("0.controller_compute");
    ROS_WARN_STREAM_THROTTLE(1, "-- Profiler report (throttled (5s)\n" << profiler_ptr_->getReport());
    ROS_INFO_STREAM_THROTTLE(1, "----------finish compute control ----------");
    controller_status_.action_ = ControllerBase::OutputAction::SEND_NOTHING;
    controller_status_.progress_ = 1.0;
    controller_status_.goal_reached_ = true;
    return controller_status_;
  }
  
  // Convert goal to fixed frame
  convertGoalToFixedFrame();

  // Check back is front to flip the pose
  checkBackIsFront();

  // Save pose as Isometry2d
  Eigen::AngleAxisd axis_angle(f_T_fb_t_.rotation());
  double theta = axis_angle.axis()(2) * axis_angle.angle();
  f_T_fb_t_2d_ = Eigen::Isometry2d::Identity();
  f_T_fb_t_2d_.translate(Eigen::Vector2d(f_T_fb_t_.translation().x(), f_T_fb_t_.translation().y()));
  f_T_fb_t_2d_.rotate(Eigen::Rotation2Dd(theta));

  // Check distance and orientation to goal
  computeDistanceAndOrientationToGoal();

  // Custom preprocess commands
  profiler_ptr_->startEvent("1.0.custom_preprocess_controller");
  customPreProcessController();
  profiler_ptr_->endEvent("1.0.custom_preprocess_controller");

  // Check controller state
  profiler_ptr_->startEvent("2.0.check_controller_state");
  checkControllerState();
  profiler_ptr_->endEvent("2.0.check_controller_state");
  // Compute control command
  profiler_ptr_->startEvent("4.0.compute_control_comand");
  computeControlCommand();
  profiler_ptr_->endEvent("4.0.compute_control_comand");
  // ROS_INFO_STREAM("pre     linear velocity: " << output_linear_velocity_.transpose() << " | angular velocity: " << output_angular_velocity_.transpose() );

  profiler_ptr_->startEvent("5.0.post_processing");
  // Enforce velocity limits and other cases
  enforceVelocityLimits();
  // ROS_INFO_STREAM("limited linear velocity: " << output_linear_velocity_.transpose() << " | angular velocity: " << output_angular_velocity_.transpose() );

  // Check if new goal is needed
  checkNewGoal();

  // Publish visualizations
  publishVisualizations(header);
  profiler_ptr_->endEvent("5.0.post_processing");


  profiler_ptr_->endEvent("0.controller_compute");
  ROS_WARN_STREAM_THROTTLE(1, "-- Profiler report (throttled (5s)\n" << profiler_ptr_->getReport());
  ROS_INFO_STREAM_THROTTLE(1, "----------finish compute control ----------");

  // Return output
  return controller_status_;
}


bool ControllerBase::checkValidGoals(){
  // if there is a goal try to follow it, otherwise return.
  if (g_Tgoal_gb_t_.translation()[0] == std::numeric_limits<double>::infinity()){
    has_got_near_to_current_goal_ = false;

    // ROS_INFO("[ControllerBase::checkValidGoals] No valid goals found");
    return false;
  }
  return true;
}

void ControllerBase::checkBackIsFront() {
  if (params_.back_is_front_) {
    Eigen::Quaterniond flip_front_direction =
        Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());
    
    // Flip base pose and goal pose
    f_T_fb_t_ = f_T_fb_t_ * flip_front_direction;
  }
}

void ControllerBase::computeDistanceAndOrientationToGoal() {
  // Compute delta time
  controller_dt_ = ts_t_ - ts_tm1_;
  ROS_INFO_STREAM_THROTTLE(1, "ts_t: " << ts_t_ << ", ts_tm1_: " << ts_tm1_ << ", dt: " << controller_dt_ );

  // set some flags to default values
  goal_closer_to_back_ = false;

  // compute robot velocity if not subscribed
  if(!use_external_velocity_) {
    Eigen::Isometry3d dT_b = f_T_fb_tm1_.inverse() * f_T_fb_t_;
    // Delta rotation to axis-angle
    Eigen::AngleAxisd axis_angle(dT_b.rotation());
    b_v_b_t_(0) = axis_angle.axis()(0) * axis_angle.angle() / controller_dt_;
    b_v_b_t_(1) = axis_angle.axis()(1) * axis_angle.angle() / controller_dt_;
    b_v_b_t_(2) = axis_angle.axis()(2) * axis_angle.angle() / controller_dt_;
    b_v_b_t_(3) = dT_b.translation().x() / controller_dt_;
    b_v_b_t_(4) = dT_b.translation().y() / controller_dt_;
    b_v_b_t_(5) = dT_b.translation().z() / controller_dt_;

    ROS_DEBUG_STREAM("estimated velocity: wx: " << b_v_b_t_(3) 
                                  << ", wy: " << b_v_b_t_(4) 
                                  << ", wz: " << b_v_b_t_(5)
                                  << ",  x: " << b_v_b_t_(0)
                                  << ",  y: " << b_v_b_t_(1)
                                  << ",  z: " << b_v_b_t_(2));
  }

    // how far are we away from the goal
  dT_base_goal_ = f_T_fb_t_.inverse() * f_Tgoal_fb_t_;
  if(params_.planar_motion_){
    dT_base_goal_.translation()[2] = 0; // not interested in z error TODO
  }
  error_distance_to_goal_ = hypot(dT_base_goal_.translation().x(), dT_base_goal_.translation().y());
  ROS_INFO_STREAM_THROTTLE(1, "x to goal: " << dT_base_goal_.translation()[0] << ", y to goal: " << dT_base_goal_.translation()[1] << ", distance to goal: " << error_distance_to_goal_);

  ////////////// If we want to track position to the goal pose
  // This is the angle, in the robot base frame from the current position to the goal position
  // We want to publish twist vectors which point in this direction
  error_heading_to_goal_ = atan2(dT_base_goal_.translation().y(), dT_base_goal_.translation().x());
  // if the goal is BEHIND, then turn to face AWAY from the goal (if enabled)
  if ((dT_base_goal_.translation().x()<0) && (params_.goal_behind_mode_ == 1 )){
    error_heading_to_goal_ = utils::mod2pi(error_heading_to_goal_ + M_PI);
    goal_closer_to_back_ = true;
  }
  ROS_INFO_STREAM_THROTTLE(1, "Heading to goal: " << error_heading_to_goal_);

  ////////////// If we want to track position back to the starting pose
  // how far are we away from the start
  Eigen::Isometry3d delta_pose_to_start = f_T_fb_t_.inverse() * f_Tstart_fb_t_ ;
  if(params_.planar_motion_)
    delta_pose_to_start.translation()[2] = 0; // not interested in z error
  // This is the angle, in the robot base frame from the current position to the starting position
  // We want to publish twist vectors which point in this direction
  error_orientation_to_starting_pose_ = std::atan2(delta_pose_to_start.translation().y(), delta_pose_to_start.translation().x());
  ROS_INFO_STREAM_THROTTLE(1, "Orientation to starting pose: " << error_orientation_to_starting_pose_);

  // Final desired (goal) orientation
  // QUaternion method
  Eigen::Quaterniond pose_quaternion(f_T_fb_t_.rotation());
  Eigen::Quaterniond goal_quaternion(f_Tgoal_fb_t_.rotation());
  Eigen::Vector3d error_orientation_to_goal_so3 = utils::subtractQuats(pose_quaternion, goal_quaternion);

  error_orientation_to_goal_ = error_orientation_to_goal_so3(2);
  ROS_INFO_STREAM_THROTTLE(1, "Orientation to goal: " << error_orientation_to_starting_pose_);

  // Compute progress so far
  // Compute distance from starting position to current goal
  Eigen::Isometry3d delta_pose_start_to_goal = f_Tstart_fb_t_.inverse() * f_Tgoal_fb_t_;
  if(params_.planar_motion_){
    delta_pose_start_to_goal.translation()[2] = 0; // not interested in z error TODO
  }
  distance_start_to_goal_ = hypot(delta_pose_start_to_goal.translation().x(), delta_pose_start_to_goal.translation().y());
}

//-------------------------------------------------------------------------------------------------
// Check unreachable state
//-------------------------------------------------------------------------------------------------
bool ControllerBase::checkUnreachableState() {
  
  bool unreachability_detected = false;

  // Compute ratio of the distances as the progress so far
  progress_t_ = std::min((distance_start_to_goal_ - error_distance_to_goal_) / distance_start_to_goal_, 1.0);
  // ROS_INFO_STREAM("[check unreachable] progress_t   " << progress_t_);
  // ROS_INFO_STREAM("[check unreachable] progress_tm1 " << progress_tm1_);

  // Compute progress step (to identify unreachable cases)
  double delta_progress = fabs(progress_t_ - progress_tm1_);
  // ROS_INFO_STREAM("[check unreachable] delta_progress " << delta_progress);
  // ROS_INFO_STREAM("[check unreachable] potential_unreachability_ " << potential_unreachability_);
  // ROS_INFO_STREAM("[check unreachable] unreachability_detected " << unreachability_detected);
  
  // If the progress seems to be stuck
  if(delta_progress <= params_.unreachability_delta_progress_threshold_) {
    if(!potential_unreachability_) {
      potential_unreachability_ = true;
      unreachability_ts_ = ts_t_;

    } else {
      if( (ts_t_ - unreachability_ts_) > params_.unreachability_time_threshold_)
        unreachability_detected = true;
    }
  } else {
    // No potential unreachable state
    potential_unreachability_ = false;
  }

  // Udpate last progress computed
  progress_tm1_ = progress_t_;

  // Return if an unreachable state was detected
  return unreachability_detected;
}


//-------------------------------------------------------------------------------------------------
// Check controller state
//-------------------------------------------------------------------------------------------------

void ControllerBase::checkControllerState() {

  // Check if we are in an unreachable state
  if(checkUnreachableState() && params_.check_unreachability_) {
    state_ = State::UNREACHABLE;
    ROS_WARN("Goal unreachable by time condition");
  }
  // Check if the goal was reached
  else if (error_distance_to_goal_ < params_.goal_distance_threshold_){
    has_got_near_to_current_goal_ = true;
    // ROS_INFO_THROTTLE(1,"TRN2DES: close to goal now. Only TRN2DES possible now"); // comment this after testing
    
    // Check the orientation error
    if ( (fabs(error_orientation_to_goal_) < params_.goal_heading_threshold_ ||
        (params_.ignore_intermediate_goal_heading_ && !goals_.empty()))){

      // The orientation error is small hence the goal was reached
      state_ = State::FINISHED;
    } else {
      if(can_rotate_) {
        // We only need to turn to reach the destination
        state_ = State::TURN_TO_DESTINATION;
      } else {
        // if we cannot rotate, we report unreachable state
        state_ = State::UNREACHABLE;
        ROS_WARN("Goal unreachable by rotation condition when close to goal 1");
      }
    }

  // Check if the goal is behind
  }else if ((dT_base_goal_.translation().x()< 0) && (params_.goal_behind_mode_ == 0)){
    state_ = State::GOAL_BEHIND;

  // Otherwise the robot needs to move
  } else {
    // TODO check this case
    // We check again if it was near before to correct heading-only
    if (has_got_near_to_current_goal_){
      // if the robot has gotten near to the goal previously, then only allow it
      // to track position and orientation
      if(can_rotate_){
        // We only need to turn to reach the destination
        state_ = State::TURN_TO_DESTINATION;
        // ROS_INFO_THROTTLE(1,"TRN2DES: Have drifted outside goal_heading_threshold_. Insisting on TRN2DES"); // comment this after testing
      } else {
        // if we cannot rotate, we report unreachable state
        state_ = State::UNREACHABLE;
        ROS_WARN("Goal unreachable by rotation condition when close to goal 2");
      }
    } else if(fabs(error_heading_to_goal_) > params_.turn_to_face_heading_threshold_ && !initial_rotation_executed_){
      state_ = State::TURN_TO_GOAL;
    } else{
      initial_rotation_executed_ = true;
      state_ = State::FORWARD;
    }
  }

  // If SHUFFLE MotionMode selected, use TURN_TO_DESTINATION instead
  if (params_.motion_mode_ == MotionMode::SHUFFLE){
    // when in the other walking states, switch to state 3 (track goal position and orientation)
    if ((state_ == State::FORWARD) || (state_ == State::TURN_TO_GOAL)){
      state_ = State::TURN_TO_DESTINATION;
    }
  }
}

void ControllerBase::checkNewGoal(){
  if (finished_with_this_goal_){
  // ROS_INFO("Get next goal");
    bool continue_commanding = getNextGoal(f_T_fb_t_);
    if (!continue_commanding){
      ROS_INFO("No new goal. Finished following goal");
      // override controller output
      // controller_status_.action_ = ControllerBase::OutputAction::SEND_STOP_WALKING;
      // controller_status_.progress_ = 0.0;
      // controller_status_.goal_reached_ = false;
    }
  }
}

//-------------------------------------------------------------------------------------------------
// Controller state machine
//-------------------------------------------------------------------------------------------------
void ControllerBase::computeControlCommand() {
  finished_with_this_goal_ = false;
  output_linear_velocity_  = Eigen::Vector3d::Zero();
  output_angular_velocity_ = Eigen::Vector3d::Zero();

  controller_status_.goal_reached_ = false;
  controller_status_.progress_ = progress_t_;

  // Handle each case
  if (state_ == State::GOAL_BEHIND) {
    ROS_INFO_STREAM_THROTTLE(1, "State: GOAL_BEHIND");
    finished_with_this_goal_ = true;
    controller_status_.action_ = computeCommandGoalBehind(output_linear_velocity_, output_angular_velocity_);

  }else if(state_ == State::TURN_TO_GOAL) {
    ROS_INFO_STREAM_THROTTLE(1, "State: TURN_TO_GOAL");
    controller_status_.action_ = computeCommandTurnToGoal(output_linear_velocity_, output_angular_velocity_);

  }else if(state_ == State::FORWARD) {
    ROS_INFO_STREAM_THROTTLE(1, "State: FORWARD");
    controller_status_.action_ = computeCommandForward(output_linear_velocity_, output_angular_velocity_);

  }else if(state_ == State::TURN_TO_DESTINATION){
    ROS_INFO_STREAM_THROTTLE(1, "State: TURN_TO_DESTINATION");
    controller_status_.action_ = computeCommandTurnToDestination(output_linear_velocity_, output_angular_velocity_);

  }else if (state_ == State::FINISHED) {
    ROS_INFO_STREAM_THROTTLE(1, "State: FINISHED");
    if(!params_.stand_safe_)
      finished_with_this_goal_ = true;
    controller_status_.action_ = computeCommandFinished(output_linear_velocity_, output_angular_velocity_);
    controller_status_.goal_reached_ = true;
    controller_status_.progress_ = 1.0;

  } else if (state_ == State::UNREACHABLE) {
    ROS_WARN_STREAM_THROTTLE(1, "State: UNREACHABLE");
    finished_with_this_goal_ = true;
    controller_status_.action_ = computeCommandFinished(output_linear_velocity_, output_angular_velocity_);
    controller_status_.action_ = OutputAction::SEND_UNREACHABLE;

  }else{
    ROS_ERROR_THROTTLE(1, "ERROR UNKOWN STATE %d", static_cast<int>(state_));
  }

  // Custom post process commands
  computeCommandPostProcess(controller_status_.action_, output_linear_velocity_, output_angular_velocity_, path_to_goal_);
}


//-------------------------------------------------------------------------------------------------
// Set robot velocity
//-------------------------------------------------------------------------------------------------
void ControllerBase::setVelocity(const Vector6d& base_velocity) {
  b_v_b_t_ = base_velocity;
  use_external_velocity_ = true;
}

//-------------------------------------------------------------------------------------------------
// Goal handling
//-------------------------------------------------------------------------------------------------
void ControllerBase::setGoal(const Eigen::Isometry3d& goal, std::string goal_frame, const Eigen::Isometry3d& current_pose){
  goals_.clear();
  goals_.push_back(goal);
  goals_frame_ = goal_frame;
  getNextGoal(current_pose);
}

// Set list of goals
void ControllerBase::setGoalList(const std::deque<Eigen::Isometry3d>& goals, std::string goal_frame, const Eigen::Isometry3d& current_pose){
  goals_.clear();
  goals_ = goals;
  goals_frame_ = goal_frame;
  getNextGoal(current_pose);
}

bool ControllerBase::getNextGoal(Eigen::Isometry3d current_pose) {
  if (goals_.size() == 0){
    // this is to try to break things which use this value
    g_Tgoal_gb_t_.translation()[0] = std::numeric_limits<double>::infinity();
    return false;
  }

  // Get the next goal
  // ROS_INFO("Goals size: %lu (before)", goals_.size());
  g_Tgoal_gb_t_ = goals_.front();
  // ROS_INFO_STREAM("Goal: " << g_Tgoal_gb_t_.matrix());
  has_got_near_to_current_goal_ = false;
  goals_.pop_front();
  f_Tstart_fb_t_ = current_pose;
  // ROS_INFO("Goals size: %lu (after)", goals_.size());
  initial_rotation_executed_ = false;

  return true;
}

void ControllerBase::stopWalking(){
  // reset to initial state
  g_Tgoal_gb_t_.translation()[0] = std::numeric_limits<double>::infinity();
  goals_.clear();
  goals_frame_ = "";
}

//-------------------------------------------------------------------------------------------------
// Utils
//-------------------------------------------------------------------------------------------------
bool ControllerBase::convertGoalToFixedFrame() {
  if (goals_frame_ == params_.fixed_frame_){
    // ROS_INFO("Goal is already in our fixed frame");
    f_Tgoal_fb_t_ = g_Tgoal_gb_t_;
    return true;

  }else { // transform goal into fixed frame
    // ROS_INFO_STREAM("Transforming goal in frame [" << goals_frame_ << "] to fixed frame [" << params_.fixed_frame_ << "]");
    tf::StampedTransform goal_to_fixed_transform;
    tf_listener_.waitForTransform(params_.fixed_frame_, goals_frame_, ros::Time(0), ros::Duration(2.0));
    try {
      tf_listener_.lookupTransform(params_.fixed_frame_, goals_frame_, ros::Time(0), goal_to_fixed_transform);

      // Convert to Isometry3d
      Eigen::Isometry3d goal_to_fixed = Eigen::Isometry3d::Identity();
      tf::transformTFToEigen (goal_to_fixed_transform, goal_to_fixed);
      
      // Update goal
      f_Tgoal_fb_t_ = goal_to_fixed * g_Tgoal_gb_t_;
      return true;
    }
    catch (tf::TransformException& ex){
      ROS_ERROR("%s",ex.what());
      return false;
    }
  }

  return false;
}

void ControllerBase::enforceVelocityLimits(){
  // Enforce velocity limits first
  // Linear
  enforceVelocityLimits(output_linear_velocity_(0),
                        params_.min_linear_velocity_,
                        params_.max_forward_linear_velocity_);
  enforceVelocityLimits(output_linear_velocity_(1),
                        params_.min_linear_velocity_,
                        params_.max_lateral_linear_velocity_);
  enforceVelocityLimits(output_linear_velocity_(2),
                        params_.min_linear_velocity_,
                        params_.max_lateral_linear_velocity_);
  // Angular                                            
  enforceVelocityLimits(output_angular_velocity_(0),
                        params_.min_angular_velocity_,
                        params_.max_angular_velocity_);
  enforceVelocityLimits(output_angular_velocity_(1),
                        params_.min_angular_velocity_,
                        params_.max_angular_velocity_);
  enforceVelocityLimits(output_angular_velocity_(2),
                        params_.min_angular_velocity_,
                        params_.max_angular_velocity_);
  
  // Enforce specific modes
  // Differential mode disregards the lateral velocity
  if (params_.differential_drive_){
    output_linear_velocity_(1) = 0.0;
  }

  // Back is front flips the control commands
  if (params_.back_is_front_){
    output_linear_velocity_ = -output_linear_velocity_;
  } 

  // Checks planar motion
  if(params_.planar_motion_){
    output_linear_velocity_(2)  = 0.0; // delete z velocity
    output_angular_velocity_(0) = 0.0; // delete roll velocity
    output_angular_velocity_(1) = 0.0; // delete pitch velocity
  }
}

void ControllerBase::enforceVelocityLimits(double& velocity, const double& min_velocity, const double& max_velocity){
  double limited_velocity = velocity;
  // Check min velocity
  if (velocity != 0 && std::fabs(velocity) < min_velocity) {
    // ROS_INFO("linear twist value changed from %f to the specified minimum of %f (with the same sign)", cmd.twist.linear.y, minLinearVelocity_);  
    //limited_velocity = (velocity < 0)? -min_velocity : min_velocity;
    limited_velocity = 0.0;
  }
  // Check max velocity
  if (std::fabs(velocity) > max_velocity) {
    // ROS_INFO("y linear twist value changed from %f to the specified minimum of %f (with the same sign)", cmd.twist.linear.y, minLinearVelocity_);  
    limited_velocity = (velocity < 0)? -max_velocity : max_velocity;
  }
  // ROS_INFO_STREAM("Limiting velocity " << velocity << " to interval [" << min_velocity << ", " << max_velocity << "] -> " << limited_velocity);

  // Output velocity
  velocity = limited_velocity;
}

//-------------------------------------------------------------------------------------------------
// Elevation map utils
//-------------------------------------------------------------------------------------------------
void ControllerBase::setElevationMap(const grid_map::GridMap& grid_map) {
  // ROS_INFO("Setting elevation map");
  // std::lock_guard<std::mutex> lock(mutex_);
  grid_map_ = grid_map;
  new_elevation_map_ = true;
}