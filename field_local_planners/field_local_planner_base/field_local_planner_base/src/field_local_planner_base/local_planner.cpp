#include <field_local_planner_base/local_planner.hpp>

namespace field_local_planner {

BaseLocalPlanner::BaseLocalPlanner() : state_(State::FINISHED), sensing_ready_(false), point_cloud_(new pcl::PointCloud<PointType>()) {}

bool BaseLocalPlanner::execute(const Time& ts, BaseLocalPlanner::Output& output) {
  if ((ts - last_ts_) < utils::fromSeconds(1 / parameters_.control_rate)) {
    return false;
  }

  // Save current timestamp
  last_ts_ = ts;

  // Check distance and orientation to goal
  computeDistanceAndOrientationToGoal();

  // Check status of the controller
  state_ = checkState();

  Twist twist;
  Path path;

  if (state_ == State::EXECUTING) {
    // Compute twist - planner specific
    twist = computeTwist();

    // Compute path
    path = computePath();
  }

  // Make output
  output.twist = twist;
  output.path = path;
  output.status.state = state_;
  output.status.distance_to_goal = distance_to_goal_;
  output.status.orientation_to_goal = orientation_to_goal_;

  return true;
}

// Other steps
void BaseLocalPlanner::computeDistanceAndOrientationToGoal() {
  // Note: All these are computed assuming planar SE(2) motion

  // Goal to current pose
  dT_b_g_ = T_f_b_.inverse() * T_f_g_;

  // Distance
  distance_to_goal_ = std::hypot(dT_b_g_.translation().y(), dT_b_g_.translation().x());
  printf("BaseLocalPlanner: distance_to_goal_: %f\n", distance_to_goal_);

  // Orientation to goal
  Vector3 rpy_goal = dT_b_g_.rotation().rpy();
  orientation_to_goal_ = rpy_goal.z();
  printf("BaseLocalPlanner: orientation_to_goal_: %f \n", orientation_to_goal_);

  // Heading to point towards goal
  heading_towards_goal_ = std::atan2(dT_b_g_.translation().y(), dT_b_g_.translation().x());
  printf("BaseLocalPlanner: heading_towards_goal_: %f \n", heading_towards_goal_);

  // Starting pose to current
  dT_b_start_ = T_f_b_.inverse() * T_f_b_start_;

  // Distance
  distance_to_start_ = std::hypot(dT_b_start_.translation().y(), dT_b_start_.translation().x());
  printf("BaseLocalPlanner: distance_to_start_: %f \n", distance_to_start_);

  // Orientation to start pose
  Vector3 rpy_start = dT_b_start_.rotation().rpy();
  orientation_to_start_ = rpy_start.z();
  printf("BaseLocalPlanner: orientation_to_start_: %f \n", orientation_to_start_);
}

BaseLocalPlanner::State BaseLocalPlanner::checkState() {
  if (parameters_.requires_sensing && !sensing_ready_) {
    std::cout << "BaseLocalPlanner: NOT_READY" << std::endl;
    return State::NOT_READY;
  }

  printf("BaseLocalPlanner: distance_to_goal_ (%f) < parameters_.distance_to_goal_thr (%f)\n", distance_to_goal_,
         parameters_.distance_to_goal_thr);
  printf("BaseLocalPlanner: orientation_to_goal_ (%f) < parameters_.orientation_to_goal_thr (%f)\n", orientation_to_goal_,
         parameters_.orientation_to_goal_thr);

  if (distance_to_goal_ < parameters_.distance_to_goal_thr && std::fabs(orientation_to_goal_) < parameters_.orientation_to_goal_thr) {
    std::cout << "BaseLocalPlanner: FINISHED" << std::endl;
    return State::FINISHED;

  } else if (checkFailure()) {
    std::cout << "BaseLocalPlanner: FAILURE" << std::endl;
    return State::FAILURE;

  } else {
    // All good, it must be working
    std::cout << "BaseLocalPlanner: same state (" << stateToStr(state_) << ")" << std::endl;
    return state_;
  }
}

bool BaseLocalPlanner::checkFailure() {
  return false;  // TODO implement proper solution
}

// Interfaces for external data
void BaseLocalPlanner::setImageRgb(const cv::Mat& img, const Pose3& T_f_s, const Time& ts) {
  image_rgb_ = img.clone();
  T_f_s_rgb_ = T_f_s;
  ts_rgb_ = ts;
  sensing_ready_ = true;
}

void BaseLocalPlanner::setImageRgbd(const cv::Mat& img, const Pose3& T_f_s, const Time& ts) {
  image_rgbd_ = img.clone();
  T_f_s_rgbd_ = T_f_s;
  ts_rgbd_ = ts;
  sensing_ready_ = true;
}

void BaseLocalPlanner::setImageDepth(const cv::Mat& img, const Pose3& T_f_s, const Time& ts) {
  image_depth_ = img.clone();
  T_f_s_depth_ = T_f_s;
  ts_depth_ = ts;
  sensing_ready_ = true;
}

void BaseLocalPlanner::setPointCloud(const pcl::PointCloud<PointType>::Ptr& cloud, const Pose3& T_f_s, const Time& ts) {
  point_cloud_ = cloud;
  T_f_s_pc_ = T_f_s;
  ts_pc_ = ts;
  sensing_ready_ = true;
}

void BaseLocalPlanner::setGridMap(const grid_map::GridMap& grid_map, const Pose3& T_f_s, const Time& ts) {
  grid_map_ = grid_map;
  T_f_s_gm_ = T_f_s;
  ts_gm_ = ts;
  sensing_ready_ = true;
}

// Set single goal
void BaseLocalPlanner::setGoalInFixed(const Pose3& T_f_g, const Pose3& T_f_b, const Time& ts) {
  T_f_g_ = T_f_g;
  T_f_b_start_ = T_f_b;
  ts_T_f_g_ = ts;
  sensing_ready_ = true;

  state_ = State::EXECUTING;
}

// Set state (pose + twist)
void BaseLocalPlanner::setPoseInFixed(const Pose3& T_f_b, const Time& ts) {
  T_f_b_ = T_f_b;
  ts_T_f_b_ = ts;
  sensing_ready_ = true;
}

void BaseLocalPlanner::setVelocityInBase(const Twist& b_v, const Time& ts) {
  b_v_ = b_v_;
  ts_b_v_ = ts;
  sensing_ready_ = true;
}

std::string BaseLocalPlanner::stateToStr(BaseLocalPlanner::State state) {
  switch (state) {
    case BaseLocalPlanner::State::NOT_READY:
      return "NOT_READY";
    case BaseLocalPlanner::State::FINISHED:
      return "FINISHED";
    case BaseLocalPlanner::State::EXECUTING:
      return "EXECUTING";
    case BaseLocalPlanner::State::FAILURE:
      return "FAILURE";
    default:
      return "INVALID_STATE";
  }
}

}  // namespace field_local_planner
