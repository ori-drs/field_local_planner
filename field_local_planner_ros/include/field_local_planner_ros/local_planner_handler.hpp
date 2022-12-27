#pragma once

#include <field_local_planner_msgs/NewGoalAction.h>
#include <field_local_planner_msgs/Status.h>
#include <field_local_planner_base/utils.hpp>
#include <field_local_planner_base_plugin/base_plugin.hpp>

#include <actionlib/server/simple_action_server.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <grid_map_msgs/GridMap.h>
#include <nav_msgs/Odometry.h>
#include <pluginlib/class_loader.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <memory>

namespace field_local_planner {

class LocalPlannerHandler {
 public:
  LocalPlannerHandler(ros::NodeHandle& nh);

 private:
  // State callbacks
  void poseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose_msg);
  void twistCallback(const geometry_msgs::TwistWithCovarianceStampedConstPtr& twist_msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr& odo_msg);

  // Sensor callbacks
  // void imageRgbCallback(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
  // void imageRgbdCallback(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::ImageConstPtr& depth_msg,
  //                        const sensor_msgs::CameraInfoConstPtr& info_msg);
  // void imageDepthCallback(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
  
  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
  void gridMapCallback(const grid_map_msgs::GridMap& cloud_msg);

  // Goal callbacks
  void goalCallback(const geometry_msgs::PoseStampedConstPtr& goal_msg);
  void joyTwistCallback(const geometry_msgs::TwistConstPtr& twist_msg);
  void newGoalRequestActionHandler();
  void preemptActionHandler();

  // Plugin loading and setup
  void loadPlugin(ros::NodeHandle& nh);
  void loadParameters(ros::NodeHandle& nh);
  void setupRos(ros::NodeHandle& nh);

  // Helpers
  void publishTwist(const geometry_msgs::Twist& twist);
  void publishZeroTwist();

 protected:
  std::string local_planner_name_;
  std::shared_ptr<pluginlib::ClassLoader<field_local_planner::BasePlugin>> plugin_loader_;
  boost::shared_ptr<field_local_planner::BasePlugin> local_planner_;  // The local planner plugin, needs to be boost:shared_ptr

  // Subscribers
  ros::Subscriber pose_sub_;
  ros::Subscriber twist_sub_;
  ros::Subscriber odometry_sub_;
  // ros::Subscriber rgb_image_sub_;
  // ros::Subscriber rgbd_image_sub_;
  // ros::Subscriber depth_image_sub_;
  ros::Subscriber point_cloud_sub_;
  ros::Subscriber grid_map_sub_;

  ros::Subscriber goal_sub_;
  ros::Subscriber joy_twist_sub_;

  // Publishers
  ros::Publisher output_twist_pub_;
  ros::Publisher status_pub_;
  std::string output_twist_type_;

  // Action server
  actionlib::SimpleActionServer<field_local_planner_msgs::NewGoalAction> action_server_;
  field_local_planner_msgs::NewGoalResult result_;
  field_local_planner_msgs::NewGoalFeedback feedback_;
};

}  // namespace field_local_planner
