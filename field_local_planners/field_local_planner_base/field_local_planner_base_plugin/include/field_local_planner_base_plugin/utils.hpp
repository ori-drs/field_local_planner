#pragma once
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <yaml-cpp/yaml.h>
#include <iostream>

namespace field_local_planner {
namespace utils {

//-------------------------------------------------------------------------------------------------
// Utils
//-------------------------------------------------------------------------------------------------
template <typename T>
T getParameter(const ros::NodeHandle& nh, const std::string& param) {
  T value;
  if (!nh.getParam(param, value)) {
    ROS_ERROR_STREAM("Could not read parameter " << param);
    exit(-1);
  }
  ROS_INFO_STREAM(param << ": " << (value));

  return value;
}

template <typename T>
T getParameterDefault(const ros::NodeHandle& nh, const std::string& param, const T& default_value) {
  T value = default_value;
  if (!nh.getParam(param, value)) {
    ROS_WARN_STREAM(param << ": " << (value) << "(default)");
  } else {
    ROS_INFO_STREAM(param << ": " << (value));
  }
  return value;
}

template <typename T>
std::vector<T> getParameterVector(const ros::NodeHandle& nh, const std::string& param) {
  std::vector<T> vector;

  XmlRpc::XmlRpcValue xml_values;
  nh.getParam(param, xml_values);
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

static inline pcl::PointXYZI createPointXYZI(float x, float y, float z, float i) {
  pcl::PointXYZI p;
  p.x = (float)x;
  p.y = (float)y;
  p.z = (float)z;
  p.intensity = (float)i;
  return p;
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

static inline void poseStampedToIsometry3dAndFrame(const geometry_msgs::PoseStampedConstPtr& msg, Eigen::Isometry3d& pose,
                                                   std::string& frame) {
  tf::poseMsgToEigen(msg->pose, pose);
  frame = msg->header.frame_id;
}

static inline void poseWithCovarianceStampedToIsometry3dAndFrame(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg,
                                                                 Eigen::Isometry3d& pose, std::string& frame) {
  tf::poseMsgToEigen(msg->pose.pose, pose);
  frame = msg->header.frame_id;
}

static inline Twist twistMsgToTwist(const geometry_msgs::TwistStampedConstPtr& msg) {
  Twist b_v = Twist::Zero();

  // We use GTSAM's convention with orientation-then-position (i.e, angular velocity and then linear)
  b_v(0) = msg->twist.angular.x;
  b_v(1) = msg->twist.angular.y;
  b_v(2) = msg->twist.angular.z;
  b_v(3) = msg->twist.linear.x;
  b_v(4) = msg->twist.linear.y;
  b_v(5) = msg->twist.linear.z;

  return b_v;
}

}  // namespace utils
}  // namespace field_local_planner