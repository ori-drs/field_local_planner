#include <locally_reactive_controller/controllers/trackline_controller.hpp>
#include <locally_reactive_controller/controllers/controller_base.hpp>
#include <math.h>

void TracklineController::customPreProcessController() {
  // init state
  state_ = State::UNKNOWN;
}

ControllerBase::OutputAction TracklineController::computeCommandGoalBehind(Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d& output_angular_velocity) {
  goal_angvel_z_ = 0;
  goal_linvel_x_ = 0;
  goal_linvel_y_ = 0;
  trackline_linvel_x_ = 0;
  trackline_linvel_y_ = 0;

  return ControllerBase::OutputAction::SEND_COMMAND;
}

ControllerBase::OutputAction TracklineController::computeCommandTurnToGoal(Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d& output_angular_velocity) {
  goal_angvel_z_ = error_heading_to_goal_ * params_.angular_gain_p_;
  
  utils::clipValue(goal_angvel_z_, params_.max_angular_velocity_);
  goal_linvel_x_ = std::cos(error_orientation_to_starting_pose_)* params_.max_forward_linear_velocity_; // m/s
  goal_linvel_y_ = std::sin(error_orientation_to_starting_pose_)* params_.max_forward_linear_velocity_; // m/s

  if (params_.yaw_exclusive_turns_) {
    goal_linvel_x_ = 0;
    goal_linvel_y_ = 0;
  }

  trackline_linvel_x_ = 0;
  trackline_linvel_y_ = 0;

  return ControllerBase::OutputAction::SEND_COMMAND;
}

ControllerBase::OutputAction TracklineController::computeCommandForward(Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d& output_angular_velocity) {
  goal_angvel_z_ = error_heading_to_goal_ * params_.angular_gain_p_;

  utils::clipValue(goal_angvel_z_, params_.max_angular_velocity_);
  goal_linvel_x_ = cos(error_heading_to_goal_)* params_.max_forward_linear_velocity_; // m/s
  goal_linvel_y_ = sin(error_heading_to_goal_)* params_.max_forward_linear_velocity_; // m/s

  // if the goal is BEHIND, then walk BACKWARDS towards the goal (if enabled)
  if ((dT_base_goal_.translation().x()<0) && (params_.goal_behind_mode_ == 1 )){
    goal_linvel_x_ = -goal_linvel_x_;
    goal_linvel_y_ = -goal_linvel_y_;
  }

  // compute trackline velocity
  computeTracklineVelocityCommand(f_Tgoal_fb_t_, f_T_fb_t_);

  return ControllerBase::OutputAction::SEND_COMMAND;
}

ControllerBase::OutputAction TracklineController::computeCommandTurnToDestination(Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d& output_angular_velocity) {
  goal_angvel_z_ = -error_orientation_to_goal_ * params_.angular_gain_p_;

  utils::clipValue(goal_angvel_z_, params_.max_angular_velocity_);
  goal_linvel_x_ = cos(error_heading_to_goal_)* params_.max_forward_linear_velocity_; // m/s
  goal_linvel_y_ = sin(error_heading_to_goal_)* params_.max_forward_linear_velocity_; // m/s

  // if the goal is BEHIND, then walk BACKWARDS towards the goal (if enabled)
  if ((dT_base_goal_.translation().x()<0) && (params_.goal_behind_mode_ == 1 )){
    goal_linvel_x_ = -goal_linvel_x_;
    goal_linvel_y_ = -goal_linvel_y_;
  }

  if (params_.yaw_exclusive_turns_) {
    goal_linvel_x_ = 0;
    goal_linvel_y_ = 0;
  }

  trackline_linvel_x_ = 0;
  trackline_linvel_y_ = 0;

  return ControllerBase::OutputAction::SEND_COMMAND;
}

ControllerBase::OutputAction TracklineController::computeCommandFinished(Eigen::Vector3d& output_linear_velocity, Eigen::Vector3d& output_angular_velocity) {
  ControllerBase::OutputAction action = ControllerBase::OutputAction::SEND_COMMAND;

  if (goals_.size() >0) {
    ROS_INFO("Continuing to walk without stopping");
    action = SEND_NOTHING;
  }

  goal_angvel_z_ = 0;
  goal_linvel_x_ = 0;
  goal_linvel_y_ = 0;


  trackline_linvel_x_ = 0;
  trackline_linvel_y_ = 0;
  
  return action;
}

void TracklineController::computeCommandPostProcess(ControllerBase::OutputAction& action, 
                                      Eigen::Vector3d& output_linear_velocity,
                                      Eigen::Vector3d& output_angular_velocity,
                                      std::vector<Eigen::Vector3d>& path_to_goal) {  
  // Compute output velocities
  double output_linvel_x = goal_linvel_x_ + trackline_linvel_x_;
  double output_linvel_y = goal_linvel_y_ + trackline_linvel_y_;

  if (params_.max_forward_linear_velocity_ < 0.01){
    // avoid dividing by zero - causes trot controller to crash
    output_linear_velocity_ = Eigen::Vector3d(0, 0, 0);
    output_angular_velocity_ = Eigen::Vector3d(0, 0, 0) ;
    action = ControllerBase::OutputAction::SEND_COMMAND;
    return;
  }

  double max_forward_linear_velocity_now = 0;
  double max_lateral_linear_velocity_now = 0;
  if ((state_ == State::FORWARD) || (params_.motion_mode_ == 1)){  // if walking straight or shuffling
    max_forward_linear_velocity_now = params_.max_forward_linear_velocity_;
    max_lateral_linear_velocity_now = params_.max_lateral_linear_velocity_;
  }else{
    max_forward_linear_velocity_now = params_.max_turning_linear_velocity_;
    max_lateral_linear_velocity_now = params_.max_turning_linear_velocity_;
  }


  double ratio_x = fabs(output_linvel_x) / max_forward_linear_velocity_now;
  if (ratio_x < 1)
    ratio_x = 1;

  output_linvel_x = output_linvel_x/ratio_x;
  output_linvel_y = output_linvel_y/ratio_x;

  double ratio_y = fabs(output_linvel_y) / max_lateral_linear_velocity_now;
  if (ratio_y < 1)
    ratio_y = 1;
  output_linvel_y = output_linvel_y/ratio_y;
  output_linvel_x = output_linvel_x/ratio_y;

  // if we are considering the robot's front as the back, need to invert the
  // control commands on the x and y axes for it to move correctly.
  output_linear_velocity = Eigen::Vector3d(output_linvel_x, output_linvel_y, 0);
  output_angular_velocity = Eigen::Vector3d(0, 0, goal_angvel_z_);

  // compute best path
  path_to_goal.clear();
  path_to_goal.push_back(Eigen::Vector3d::Zero());
  path_to_goal.push_back(dT_base_goal_.translation());
}

void TracklineController::computeTracklineVelocityCommand(Eigen::Isometry3d current_goal, Eigen::Isometry3d current_pose){
  // How far is the robot from the trackline?
  // Create an x/y base velocity term proportional to that distance 

  // signed perpendicular distance between pt0 and line (between pt1 and pt2)
  // point: 0  current_pose
  // line: 1  starting_pose and  2  current_goal
	// distance is negative on LHS of line. positive on RHS of line
  double x0, y0, x1, y1, x2, y2;
  x0 = current_pose.translation().x();
  y0 = current_pose.translation().y();
  x1 = f_Tstart_fb_t_.translation().x();
  y1 = f_Tstart_fb_t_.translation().y();
  x2 = current_goal.translation().x();
  y2 = current_goal.translation().y();
  double numer = (x2-x1)*(y1-y0) - (x1-x0)*(y2-y1);
  double denom = sqrt( (y2-y1)*(y2-y1) + (x2-x1)*(x2-x1) );
  double trackline_distance_error = numer / denom;
  double slope = atan2((y2 - y1),(x2 - x1));
  //ROS_INFO_THROTTLE(1,"        TRACKLN: %f slope of trackline", slope*57);

  Eigen::Vector3d current_euler = utils::getEulerAngles(current_pose.rotation());
  double current_yaw = current_euler(2);
  //ROS_INFO_THROTTLE(1,"        TRACKLN: %f current_yaw of robot", current_yaw*57);

  // This is the angle between the trackline and the direction the robot is facing
  // this is zero if the robot is on trackline and facing goal
  // it is negative on LHS of line. positive on RHS of line
  double projection_angle = current_yaw - slope;
  // ROS_INFO_THROTTLE(1,"        TRACKLN: %f projection_angle", projection_angle*57);
  

  // Create velocity command proportional to error
  double trackline_velocity = trackline_distance_error * params_.linear_gain_p_;
  if (trackline_velocity > params_.max_forward_linear_velocity_)
    trackline_velocity = params_.max_forward_linear_velocity_;
  if (trackline_velocity < -params_.max_forward_linear_velocity_)
    trackline_velocity = -params_.max_forward_linear_velocity_;

  trackline_linvel_x_ =  trackline_velocity * sin(projection_angle);
  trackline_linvel_y_ =  trackline_velocity * cos(projection_angle);
  //ROS_INFO_THROTTLE(1,"        TRACKLN: %f      Velocity norm", trackline_velocity);
  //ROS_INFO_THROTTLE(1,"        TRACKLN: %f  Forward  %f  Left", trackline_linvel_x_, trackline_linvel_y_);
}

