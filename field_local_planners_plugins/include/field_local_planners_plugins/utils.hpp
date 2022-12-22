#pragma once
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <yaml-cpp/yaml.h>

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>

#include <iostream>

namespace field_local_planners {
namespace utils {

#define COL_WIDTH 40

//-------------------------------------------------------------------------------------------------
// Utils
//-------------------------------------------------------------------------------------------------
template <typename T>
T getParameter(const ros::NodeHandle& node_handle, const std::string& param) {
  T value;
  if (!node_handle.getParam(param, value)) {
    ROS_ERROR_STREAM("Could not read parameter " << param);
    exit(-1);
  }
  ROS_INFO_STREAM(param << ": " << (value));

  return value;
}

template <typename T>
T getParameterDefault(const ros::NodeHandle& node_handle, const std::string& param, const T& default_value) {
  T value = default_value;
  if (!node_handle.getParam(param, value)) {
    ROS_WARN_STREAM(param << ": " << (value) << "(default)");
  } else {
    ROS_INFO_STREAM(param << ": " << (value));
  }
  return value;
}

template <typename T>
std::vector<T> getParameterVector(const ros::NodeHandle& node_handle, const std::string& param) {
  std::vector<T> vector;

  XmlRpc::XmlRpcValue xml_values;
  node_handle.getParam(param, xml_values);
  if (xml_values.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ERROR_STREAM(param << " parameter must be a list.");
    exit(-1);
  } else {
    for (int i = 0; i < xml_values.size(); i++) {
      vector.push_back(static_cast<T>(xml_values[i]));
    }
  }

  return vector;
}

template <typename T>
static void assignAndPrintDiff(std::string var_name, T& var, const T& new_val) {
  if (var != new_val) {
    ROS_WARN_STREAM("[" << var_name << "] changed from " << static_cast<T>(var) << " to " << static_cast<T>(new_val));
  }
  var = new_val;
}

const inline int64_t getHeaderUTime(const std_msgs::Header& header) {
  return (int64_t)floor(header.stamp.toNSec() / 1000);
}

//-------------------------------------------------------------------------------------------------
// Conversions
//-------------------------------------------------------------------------------------------------
static inline void poseStampedToPose3AndFrame(const geometry_msgs::PoseStampedConstPtr& msg, Pose3& pose, std::string& frame) {
  Eigen::Isometry3d eigen_pose;
  tf::poseMsgToEigen(msg->pose, eigen_pose);
  pose = Pose3(eigen_pose.matrix());

  frame = msg->header.frame_id;
}

static inline void poseWithCovarianceStampedToPose3AndFrame(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg, Pose3& pose,
                                                            std::string& frame) {
  Eigen::Isometry3d eigen_pose;
  tf::poseMsgToEigen(msg->pose.pose, eigen_pose);
  pose = Pose3(eigen_pose.matrix());

  frame = msg->header.frame_id;
}

}  // namespace utils
}  // namespace field_local_planners