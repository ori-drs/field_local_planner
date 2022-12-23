#pragma once
#include <field_local_planner_base/local_planner.hpp>
#include <field_local_planner_base_plugin/utils.hpp>

#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <grid_map_msgs/GridMap.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <memory>

namespace field_local_planner {

class BasePlugin {
 protected:
  BasePlugin() {}

 public:
  // This method needs to be defined by the other plugins
  virtual void loadParameters(ros::NodeHandle& nh) = 0;
  virtual void setupRos(ros::NodeHandle& nh) = 0;

  // Initializes the plugin
  void initialize(ros::NodeHandle& nh) {
    // Load basic parameters
    commonSetup(nh);

    // Load local planner parameters
    loadParameters(nh);

    // Setup ROS publishers and subscribers
    setupRos(nh);
  }

  void commonSetup(ros::NodeHandle& nh) {
    fixed_frame_ = utils::getParameter<std::string>(nh, "fixed_frame");
    base_frame_ = utils::getParameter<std::string>(nh, "base_frame");
  }

  void setPose(const geometry_msgs::Pose& pose_msg, const std_msgs::Header& header = std_msgs::Header());
  void setVelocity(const geometry_msgs::Twist& twist_msg, const std_msgs::Header& header = std_msgs::Header());
  void setGoal(const geometry_msgs::Pose& pose_msg, const std_msgs::Header& header = std_msgs::Header());

  // Interfaces for external data
  void setImageRgb(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
  void setImageRgbd(const sensor_msgs::ImageConstPtr& rgb_msg, const sensor_msgs::ImageConstPtr& depth_msg,
                    const sensor_msgs::CameraInfoConstPtr& info_msg);
  void setImageDepth(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
  void setPointCloud(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
  void setGridMap(const grid_map_msgs::GridMap& cloud_msg);



  void setJoyCommand(const geometry_msgs::Twist& twist_msg);

  std::string getFixedFrame() const { return fixed_frame_; };
  std::string getBaseFrame() const { return base_frame_; };

 protected:
  std::shared_ptr<BaseLocalPlanner> local_planner_;  // The actual local planner

  // TF
  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;

  // Frames
  std::string fixed_frame_;  // usually 'odom' frame
  std::string base_frame_;   // usually 'base' frame
};

}  // namespace field_local_planner
