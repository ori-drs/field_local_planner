#include <field_local_planner_base_plugin/plugin.hpp>

namespace field_local_planner {

void BasePlugin::setPose(const geometry_msgs::Pose& pose_msg, const std_msgs::Header& header) {
  // local_planner_->setPoseInFixed(T_f_b);
}

void BasePlugin::setVelocity(const geometry_msgs::Twist& twist_msg, const std_msgs::Header& header) {
  // local_planner_->setVelocityInBase(b_v);
}

void BasePlugin::setGoal(const geometry_msgs::Pose& pose_msg, const std_msgs::Header& header) {
  // Update goal tf
  // tf::Transform goal_transform;
  // tf::transformEigenToTF(goal, goal_transform);
  // goal_transform.setRotation(goal_transform.getRotation().normalize());
  // tf_broadcaster_.sendTransform(tf::StampedTransform(goal_transform, ros::Time::now(), fixed_frame_, "goal_field_local_planner"));

  // local_planner_->setGoalInFixed(T_f_g, T_f_b);
}


void BasePlugin::setImageRgb(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::CameraInfoConstPtr& info_msg) {
  // local_planner_->setImageRgb(img, T_b_s);
}

void BasePlugin::setImageRgbd(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::ImageConstPtr& depth_msg,
                              const sensor_msgs::CameraInfoConstPtr& info_msg) {
  // local_planner_->setImageRgbd(img, T_b_s);
}

void BasePlugin::setImageDepth(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::CameraInfoConstPtr& info_msg) {
  // local_planner_->setImageDepth(img, T_b_s);
}

void BasePlugin::setPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
  // local_planner_->setPointCloud(cloud, T_b_s);
  // local_planner_->setFixedToMapTransform(T_m_f);
}

void BasePlugin::setGridMap(const grid_map_msgs::GridMap& cloud_msg) {
  // local_planner_->setGridMap(grid_map, T_b_s);
  // local_planner_->setFixedToMapTransform(T_m_f);
}

void BasePlugin::setJoyCommand(const geometry_msgs::Twist& twist_msg) {
  //
}

}  // namespace field_local_planner