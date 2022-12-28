#pragma once
#include <field_local_planner_msgs/Status.h>
#include <field_local_planner_base/local_planner.hpp>
#include <field_local_planner_base_plugin/utils.hpp>

#include <dynamic_reconfigure/server.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <grid_map_msgs/GridMap.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <grid_map_ros/grid_map_ros.hpp>

#include <memory>

namespace field_local_planner {

class BasePlugin {
 protected:
  BasePlugin() {}

 public:
  // This method needs to be defined by the other plugins
  virtual void loadParameters(ros::NodeHandle& nh) = 0;
  virtual void setupRos(ros::NodeHandle& nh) = 0;
  virtual void publishVisualizations() = 0;

  // Initializes the plugin
  void initialize(ros::NodeHandle& nh);
  void loadBaseParameters(ros::NodeHandle& nh);
  bool execute(const ros::Time& stamp, geometry_msgs::Twist& twist_msg, nav_msgs::Path& path, field_local_planner_msgs::Status& status_msg);

 private:
  // Utils
  Pose3 queryTransform(const std::string& parent, const std::string& child, const ros::Time& stamp = ros::Time::now());
  void publishTransform(const Pose3& T_parent_child, const std::string& parent, const std::string& child,
                        const ros::Time& stamp = ros::Time::now());
  bool isValidFrame(const std::string& frame) const;

 public:
  // Interfaces for external data
  void setPose(const geometry_msgs::Pose& pose_msg, const std_msgs::Header& header = std_msgs::Header());
  void setVelocity(const geometry_msgs::Twist& twist_msg, const std_msgs::Header& header = std_msgs::Header());
  void setGoal(const geometry_msgs::Pose& goal_msg, const std_msgs::Header& header = std_msgs::Header());

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
  std::vector<std::string> valid_goal_frames_;
};

}  // namespace field_local_planner
