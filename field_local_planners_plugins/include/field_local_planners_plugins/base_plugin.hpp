#pragma once
#include <dynamic_reconfigure/server.h>
#include <grid_map_msgs/GridMap.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <field_local_planners/base_local_planner.hpp>
#include <field_local_planners_plugins/utils.hpp>
#include <memory>
namespace field_local_planners {

class BasePlugin {
 protected:
  BasePlugin() {}

 public:
  // This method needs to be defined by the other plugins
  virtual void loadParameters(ros::NodeHandle& nh) = 0;
  virtual void setupRos(ros::NodeHandle& nh) = 0;

  // Initializes the plugin
  void initialize(ros::NodeHandle& nh) {
    // Load local planner parameters
    loadParameters(nh);

    // Setup ROS publishers and subscribers
    setupRos(nh);

    ROS_INFO("Initialized");
  }

  // Interfaces for external data
  void setImageRgb(const cv::Mat& img, const Pose3& T_b_s) { local_planner_->setImageRgb(img, T_b_s); }
  void setImageRgbd(const cv::Mat& img, const Pose3& T_b_s) { local_planner_->setImageRgbd(img, T_b_s); }
  void setImageDepth(const cv::Mat& img, const Pose3& T_b_s) { local_planner_->setImageDepth(img, T_b_s); }
  void setPointCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, const Pose3& T_b_s) { local_planner_->setPointCloud(cloud, T_b_s); }
  void setGridMap(const grid_map::GridMap& grid_map, const Pose3& T_b_s) { local_planner_->setGridMap(grid_map, T_b_s); }
  void setPoseInFixed(const Pose3& T_f_b) { local_planner_->setPoseInFixed(T_f_b); }
  void setVelocityInBase(const Twist& b_v) { local_planner_->setVelocityInBase(b_v); }
  void setFixedToMapTransform(const Pose3& T_m_f) { local_planner_->setFixedToMapTransform(T_m_f); }
  void setGoalInFixed(const Pose3& T_f_g, const Pose3& T_f_b) { local_planner_->setGoalInFixed(T_f_g, T_f_b); }

 protected:
  std::shared_ptr<BaseLocalPlanner> local_planner_;  // The actual local planner
};

}  // namespace field_local_planners
