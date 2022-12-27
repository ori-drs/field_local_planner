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
}

void BasePlugin::execute(geometry_msgs::Twist& twist_msg, field_local_planner_msgs::Status& status_msg) {
  // Compute output
  BaseLocalPlanner::Output output = local_planner_->execute();

  // Convert to ROS msgs
  twist_msg = utils::toTwistMsg(output.twist);
  status_msg = utils::toStatusMsg(output.status);
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

void BasePlugin::setPose(const geometry_msgs::Pose& pose_msg, const std_msgs::Header& header) {
  // Get transformation if frames do not match
  // frame_id (auxiliary) in fixed_frame f
  Pose3 T_f_a = queryTransform(fixed_frame_, header.frame_id, header.stamp);

  // Transform message
  Pose3 T_a_b = utils::toPose3(pose_msg);  // Transformation of base b in auxiliary frame
  Pose3 T_f_b = T_f_a * T_a_b;             // Base in fixed frame

  // Set data
  local_planner_->setPoseInFixed(T_f_b);
}

void BasePlugin::setVelocity(const geometry_msgs::Twist& twist_msg, const std_msgs::Header& header) {
  Twist twist = utils::toTwist(twist_msg);
  local_planner_->setVelocityInBase(twist);
}

void BasePlugin::setGoal(const geometry_msgs::Pose& goal_msg, const std_msgs::Header& header) {
  // Get transformation if frames do not match
  // frame_id (auxiliary) in fixed_frame f
  Pose3 T_f_a = queryTransform(fixed_frame_, header.frame_id, header.stamp);

  // Transform message
  Pose3 T_a_g = utils::toPose3(goal_msg);  // Transformation of base b in auxiliary frame
  Pose3 T_f_g = T_f_a * T_a_g;             // Base in fixed frame

  // Query base in fixed frame
  Pose3 T_f_b = queryTransform(fixed_frame_, base_frame_, header.stamp);

  // Set data
  local_planner_->setGoalInFixed(T_f_g, T_f_b);
}

void BasePlugin::setImageRgb(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::CameraInfoConstPtr& info_msg) {
  // local_planner_->setImageRgb(img, T_b_s);
  ROS_FATAL("BasePlugin::setImageRgb: Not implemented");
}

void BasePlugin::setImageRgbd(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::ImageConstPtr& depth_msg,
                              const sensor_msgs::CameraInfoConstPtr& info_msg) {
  // local_planner_->setImageRgbd(img, T_b_s);
  ROS_FATAL("BasePlugin::setImageRgbd: Not implemented");
}

void BasePlugin::setImageDepth(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::CameraInfoConstPtr& info_msg) {
  // local_planner_->setImageDepth(img, T_b_s);
  ROS_FATAL("BasePlugin::setImageDepth: Not implemented");
}

void BasePlugin::setPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
  // local_planner_->setPointCloud(cloud, T_b_s);
  // local_planner_->setFixedToMapTransform(T_m_f);
}

void BasePlugin::setGridMap(const grid_map_msgs::GridMap& grid_map_msg) {
  // Convert grid map
  grid_map::GridMap grid_map;
  grid_map::GridMapRosConverter::fromMessage(grid_map_msg, grid_map);
  grid_map.convertToDefaultStartIndex();

  // Query transformation from grid map frame
  Pose3 T_f_m = queryTransform(fixed_frame_, grid_map.getFrameId(), grid_map_msg.info.header.stamp);

  // Set grid map in local planner
  local_planner_->setGridMap(grid_map, T_f_m);
}

void BasePlugin::setJoyCommand(const geometry_msgs::Twist& twist_msg) {
  //
}

}  // namespace field_local_planner