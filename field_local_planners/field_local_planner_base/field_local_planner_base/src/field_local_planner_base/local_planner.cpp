#include <field_local_planner_base/local_planner.hpp>

namespace field_local_planner {

BaseLocalPlanner::BaseLocalPlanner() : sensing_ready_(false), point_cloud_(new pcl::PointCloud<pcl::PointXYZI>()) {}

bool BaseLocalPlanner::execute(const Time& ts, BaseLocalPlanner::Output& output) {
  if ((ts - last_ts_) < utils::fromSeconds(1 / parameters_.control_rate)) {
    return false;
  }
  
  // Save current timestamp
  last_ts_ = ts;

  // Check distance and orientation to goal
  computeDistanceAndOrientationToGoal();

  // Check status of the controller
  Status status = checkStatus();

  // Compute twist - planner specific
  Twist twist = computeTwist();

  // Compute path
  Path path = computePath();

  // Make output
  output.twist = twist;
  output.path = path;
  output.status = status;

  return true;
}

// Other steps
void BaseLocalPlanner::computeDistanceAndOrientationToGoal() {
  // Goal to current pose
  dT_b_g_ = T_f_b_.inverse() * T_f_g_;

  // Distance
  distance_to_goal_ = dT_b_g_.translation().norm();

  // Orientation to goal
  Vector3 rpy_goal = dT_b_g_.rotation().rpy();
  orientation_to_goal_ = rpy_goal.z();

  // Heading to point towards goal
  heading_towards_goal_ = atan2(dT_b_g_.translation().y(), dT_b_g_.translation().x());

  // Starting pose to current
  dT_b_start_ = T_f_b_.inverse() * T_f_b_start_;

  // Distance
  distance_to_start_ = dT_b_start_.translation().norm();

  // Orientation to start pose
  Vector3 rpy_start = dT_b_start_.rotation().rpy();
  orientation_to_start_ = rpy_start.z();
}

BaseLocalPlanner::Status BaseLocalPlanner::checkStatus() {
  if (parameters_.requires_sensing && !sensing_ready_) {
    return Status::NOT_READY;
  }

  if (distance_to_goal_ < parameters_.distance_to_goal_thr && orientation_to_goal_ < parameters_.orientation_to_goal_thr) {
    return Status::FINISHED;

  } else if (checkFailure()) {
    return Status::FAILURE;

  } else {
    // All good, it must be working
    return Status::EXECUTING;
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

void BaseLocalPlanner::setPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const Pose3& T_f_s, const Time& ts) {
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

}  // namespace field_local_planner
