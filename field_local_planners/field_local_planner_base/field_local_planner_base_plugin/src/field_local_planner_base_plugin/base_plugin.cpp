#include <field_local_planner_base_plugin/base_plugin.hpp>

namespace field_local_planner {

void BasePlugin::initialize(ros::NodeHandle& nh) {
  // Load basic parameters
  loadBaseParameters(nh);

  // Load local planner parameters
  loadParameters(nh);

  // Setup ROS publishers and subscribers
  setupRos(nh);
}

void BasePlugin::loadBaseParameters(ros::NodeHandle& nh) {
  fixed_frame_ = utils::getParameter<std::string>(nh, "fixed_frame");
  base_frame_ = utils::getParameter<std::string>(nh, "base_frame");
  valid_goal_frames_ = utils::getParameterVector<std::string>(nh, "valid_goal_frames");

  grid_map_to_cloud_ = utils::getParameter<bool>(nh, "grid_map_to_cloud");
  grid_map_to_cloud_range_ = utils::getParameter<double>(nh, "grid_map_to_cloud_range");
  grid_map_to_cloud_filter_size_ = utils::getParameter<double>(nh, "grid_map_to_cloud_filter_size");

  voxel_filter_.setLeafSize(grid_map_to_cloud_filter_size_, grid_map_to_cloud_filter_size_, grid_map_to_cloud_filter_size_);
}

bool BasePlugin::execute(const ros::Time& stamp, geometry_msgs::Twist& twist_msg, nav_msgs::Path& path_msg,
                         field_local_planner_msgs::Status& status_msg) {
  // Convert stamp
  Time ts = utils::toTimeStamp(stamp);

  // Compute output
  BaseLocalPlanner::Output output;
  bool new_output = local_planner_->execute(ts, output);

  if (new_output) {
    // Convert to ROS msgs
    twist_msg = utils::toTwistMsg(output.twist);
    status_msg = utils::toStatusMsg(output.status);
    path_msg = utils::toPathMsg(output.path, base_frame_);

    // Publish visualizations
    publishVisualizations();
  }

  return new_output;
}

Pose3 BasePlugin::queryTransform(const std::string& parent, const std::string& child, const ros::Time& stamp) {
  tf::StampedTransform T_parent_child;
  Eigen::Isometry3d eigen_T_parent_child = Eigen::Isometry3d::Identity();

  tf_listener_.waitForTransform(parent, child, stamp, ros::Duration(1.0));
  try {
    tf_listener_.lookupTransform(parent, child, stamp, T_parent_child);
    tf::transformTFToEigen(T_parent_child, eigen_T_parent_child);

  } catch (tf::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
  }

  // Return in Pose3 format
  return Pose3(eigen_T_parent_child.matrix());
}

void BasePlugin::publishTransform(const Pose3& T_parent_child, const std::string& parent, const std::string& child,
                                  const ros::Time& stamp) {
  tf::Transform tf_parent_child = utils::toTfTransform(T_parent_child);
  tf_broadcaster_.sendTransform(tf::StampedTransform(tf_parent_child, stamp, parent, child));
}

bool BasePlugin::isValidFrame(const std::string& frame) const {
  return std::find(valid_goal_frames_.begin(), valid_goal_frames_.end(), frame) != valid_goal_frames_.end();
}

void BasePlugin::getPointCloudFromGridMap(const grid_map::GridMap& grid_map, pcl::PointCloud<PointType>::Ptr& cloud, Pose3& T_f_s) {
  grid_map::GridMap cropped_grid_map;
  grid_map::Position position(grid_map.getPosition().x(), grid_map.getPosition().y());
  grid_map::Length length(2.0 * grid_map_to_cloud_range_, 2.0 * grid_map_to_cloud_range_);

  bool success;
  cropped_grid_map = grid_map.getSubmap(position, length, success);

  // Convert to point_cloud
  pcl::PointCloud<PointType>::Ptr tmp_cloud(new pcl::PointCloud<PointType>());

  sensor_msgs::PointCloud2 point_cloud;
  grid_map::GridMapRosConverter::toPointCloud(cropped_grid_map, {"elevation", "traversability"}, "elevation", point_cloud);
  pcl::fromROSMsg(point_cloud, *tmp_cloud);

  // Filter using voxel_filter
  voxel_filter_.setInputCloud(tmp_cloud);
  voxel_filter_.filter(*tmp_cloud);

  // Transform to base frame
  Pose3 T_b_m = queryTransform(base_frame_, grid_map.getFrameId());
  pcl::transformPointCloud(*tmp_cloud, *cloud, T_b_m.matrix());
  T_f_s = queryTransform(fixed_frame_, base_frame_);
}

void BasePlugin::setPose(const geometry_msgs::Pose& pose_msg, const std_msgs::Header& header) {
  // Convert stamp
  Time ts = utils::toTimeStamp(header.stamp);

  // Get transformation if frames do not match
  // frame_id (auxiliary) in fixed_frame f
  Pose3 T_f_a = queryTransform(fixed_frame_, header.frame_id, header.stamp);

  // Transform message
  Pose3 T_a_b = utils::toPose3(pose_msg);  // Transformation of base b in auxiliary frame
  Pose3 T_f_b = T_f_a * T_a_b;             // Base in fixed frame

  // Set data
  local_planner_->setPoseInFixed(T_f_b, ts);
}

void BasePlugin::setVelocity(const geometry_msgs::Twist& twist_msg, const std_msgs::Header& header) {
  // Convert stamp
  Time ts = utils::toTimeStamp(header.stamp);

  Twist twist = utils::toTwist(twist_msg);
  local_planner_->setVelocityInBase(twist, ts);
}

void BasePlugin::setGoal(const geometry_msgs::Pose& goal_msg, const std_msgs::Header& header) {
  // Convert stamp
  Time ts = utils::toTimeStamp(header.stamp);

  // Check if goal is defined in a valid frame
  if (!isValidFrame(header.frame_id)) {
    ROS_ERROR_STREAM("setGoal: Frame [" << header.frame_id << "] is not valid.");
    return;
  }

  // Get transformation if frames do not match
  // frame_id (auxiliary) in fixed_frame f
  Pose3 T_f_a = queryTransform(fixed_frame_, header.frame_id, header.stamp);

  // Transform message
  Pose3 T_a_g = utils::toPose3(goal_msg);  // Transformation of base b in auxiliary frame
  Pose3 T_f_g = T_f_a * T_a_g;             // Base in fixed frame
  publishTransform(T_f_g, fixed_frame_, "goal_local_planner");

  // Query base in fixed frame
  Pose3 T_f_b = queryTransform(fixed_frame_, base_frame_, header.stamp);

  // Set data
  ROS_INFO_STREAM("Setting new goal in frame [" << fixed_frame_ << "] : \n" << T_f_g);
  local_planner_->setGoalInFixed(T_f_g, T_f_b, ts);
}

void BasePlugin::setImageRgb(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::CameraInfoConstPtr& info_msg) {
  // Convert stamp
  Time ts = utils::toTimeStamp(info_msg->header.stamp);

  // local_planner_->setImageRgb(img, T_b_s);
  ROS_FATAL("BasePlugin::setImageRgb: Not implemented");
}

void BasePlugin::setImageRgbd(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::ImageConstPtr& depth_msg,
                              const sensor_msgs::CameraInfoConstPtr& info_msg) {
  // Convert stamp
  Time ts = utils::toTimeStamp(info_msg->header.stamp);
  // local_planner_->setImageRgbd(img, T_b_s);
  ROS_FATAL("BasePlugin::setImageRgbd: Not implemented");
}

void BasePlugin::setImageDepth(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::CameraInfoConstPtr& info_msg) {
  // Convert stamp
  Time ts = utils::toTimeStamp(info_msg->header.stamp);
  // local_planner_->setImageDepth(img, T_b_s);
  ROS_FATAL("BasePlugin::setImageDepth: Not implemented");
}

void BasePlugin::setPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
  // Convert stamp
  Time ts = utils::toTimeStamp(cloud_msg->header.stamp);
  // local_planner_->setPointCloud(cloud, T_b_s);
  ROS_FATAL("BasePlugin::setPointCloud: Not implemented");
}

void BasePlugin::setGridMap(const grid_map_msgs::GridMap& grid_map_msg) {
  // Convert stamp
  Time ts = utils::toTimeStamp(grid_map_msg.info.header.stamp);

  // Convert grid map
  grid_map::GridMap grid_map;
  grid_map::GridMapRosConverter::fromMessage(grid_map_msg, grid_map);
  grid_map.convertToDefaultStartIndex();

  // Query transformation from grid map frame
  Pose3 T_f_m = queryTransform(fixed_frame_, grid_map.getFrameId(), grid_map_msg.info.header.stamp);

  // Set grid map in local planner
  local_planner_->setGridMap(grid_map, T_f_m, ts);

  if (grid_map_to_cloud_) {
    pcl::PointCloud<PointType>::Ptr cloud;
    Pose3 T_f_s;
    getPointCloudFromGridMap(grid_map, cloud, T_f_s);

    local_planner_->setPointCloud(cloud, T_f_s, ts);
  }
}

void BasePlugin::setJoyCommand(const geometry_msgs::Twist& twist_msg) {
  //
  ROS_FATAL("BasePlugin::setJoyCommand: Not implemented");
}

}  // namespace field_local_planner