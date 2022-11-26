#pragma once
// #include <ros/ros.h>
// #include <ros/console.h>
// #include <tf/tf.h>
// #include <eigen_conversions/eigen_msg.h>
// #include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <yaml-cpp/yaml.h>
#include <iostream>

namespace utils{

#define COL_WIDTH 40

// //-------------------------------------------------------------------------------------------------
// // Utils
// //-------------------------------------------------------------------------------------------------
// template<typename T>
// T getParameter(const ros::NodeHandle& node_handle, const std::string &param) {
//   T value;
//   if (!node_handle.getParam(param, value)) {
//     ROS_ERROR_STREAM("Could not read parameter " << param);
//     exit(-1);
//   }
//   ROS_INFO_STREAM(param << ": " << (value));

//   return value;
// }

// // The methods below were defined here to use the node_handle
// // if they are moved to utils.hpp the input will be too verbose

// template<typename T>
// T getParameterDefault(const ros::NodeHandle& node_handle, const std::string &param, const T &default_value) {
//   T value = default_value;
//   if (!node_handle.getParam(param, value)) {
//     ROS_WARN_STREAM(param << ": " << (value) << "(default)");
//   } else{
//     ROS_INFO_STREAM(param << ": " << (value));
//   }
//   return value;
// }

// template<typename T>
// std::vector<T> getParameterVector(const ros::NodeHandle& node_handle, const std::string &param) {
//   std::vector<T> vector;

//   XmlRpc::XmlRpcValue xml_values;
//   node_handle.getParam(param, xml_values);
//   if (xml_values.getType() != XmlRpc::XmlRpcValue::TypeArray) {
//     ROS_ERROR_STREAM(param << " parameter must be a list.");
//     exit(-1);
//   } else {
//     for(int i =0; i <xml_values.size(); i++)
//     {
//       vector.push_back(static_cast<T>(xml_values[i]));
//     }
//   }

//   return vector;
// }

/// @brief Wrapper for yaml-cpp to provide default config files if not provided.
/// @author David Wisth
template <typename T> T get(const YAML::Node &node, const int &param, const T &default_value, bool silent=false) {
  if (!node[param]) {
    //if(!silent) std::cout << std::left << std::setw(COL_WIDTH) << param << ": Not found. Using default value: " << default_value << std::endl;
    return default_value;
  }
  //if(!silent) std::cout << std::left << std::setw(COL_WIDTH) << ": " << node[param].as<T>() << std::endl;
  return node[param].as<T>();
}

template <typename T> T get(const YAML::Node &node, const std::string &param, const T &default_value, bool silent=false) {
  if (!node[param]) {
    //if(!silent) std::cout << std::left << std::setw(COL_WIDTH) << param << ": Not found. Using default value: " << default_value << std::endl;
    return default_value;
  }
  //if(!silent) std::cout << std::left << std::setw(COL_WIDTH)  << param << ": " << node[param].as<T>() << std::endl;
  return node[param].as<T>();
}

// static inline pcl::PointXYZI createPointXYZI(float x, float y, float z, float i) {
//   pcl::PointXYZI p;
//   p.x = (float)x;
//   p.y = (float)y;
//   p.z = (float)z;
//   p.intensity = (float)i;
//   return p;
// }

// const inline int64_t getHeaderUTime(const std_msgs::Header& header){
//   return (int64_t)floor(header.stamp.toNSec() / 1000);
// }

static inline double clipValue(double value, double max_value){
  return std::max(std::min(value, max_value), -max_value);
}

// constrain angle to be -180:180 in radians
static inline double constrainAngle(double x){
  x = fmod(x + M_PI,2.0*M_PI);
  if (x < 0)
      x += 2.0*M_PI;
  return x - M_PI;
}

// static inline void poseStampedToIsometry3dAndFrame(const geometry_msgs::PoseStampedConstPtr& msg, Eigen::Isometry3d& pose, std::string& frame) {
//   tf::poseMsgToEigen(msg->pose, pose);
//   frame = msg->header.frame_id;
// }

// static inline void poseWithCovarianceStampedToIsometry3dAndFrame(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg, Eigen::Isometry3d& pose, std::string& frame) {
//   tf::poseMsgToEigen(msg->pose.pose, pose);
//   frame = msg->header.frame_id;
// }

// template<typename T>
// static void assignAndPrintDiff(std::string var_name, T& var, const T& new_val){
//   if (var != new_val){
//     ROS_WARN_STREAM("[" << var_name << "] changed from " << static_cast<T>(var) << " to " << static_cast<T>(new_val));
//   }
//   var = new_val;
// }

/**
 * Convert the angle given in radians to degrees.
 * https://techoverflow.net/2019/05/30/c-equivalent-of-numpy-php-rad2deg/
 */
template <typename T> static T rad2deg(T angle) { return angle * 180.0 / M_PI; }
template <typename T> static T deg2rad(T angle) { return angle / 180.0 * M_PI; }


// Bessel function 1st kind, order 0
// from https://www.atnf.csiro.au/computing/software/gipsy/sub/bessel.c
static double bessj0( double x )
{
   double ax,z;
   double xx,y,ans,ans1,ans2;

   if ((ax=fabs(x)) < 8.0) {
      y=x*x;
      ans1=57568490574.0+y*(-13362590354.0+y*(651619640.7
         +y*(-11214424.18+y*(77392.33017+y*(-184.9052456)))));
      ans2=57568490411.0+y*(1029532985.0+y*(9494680.718
         +y*(59272.64853+y*(267.8532712+y*1.0))));
      ans=ans1/ans2;
   } else {
      z=8.0/ax;
      y=z*z;
      xx=ax-0.785398164;
      ans1=1.0+y*(-0.1098628627e-2+y*(0.2734510407e-4
         +y*(-0.2073370639e-5+y*0.2093887211e-6)));
      ans2 = -0.1562499995e-1+y*(0.1430488765e-3
         +y*(-0.6911147651e-5+y*(0.7621095161e-6
         -y*0.934935152e-7)));
      ans=sqrt(0.636619772/ax)*(cos(xx)*ans1-z*sin(xx)*ans2);
   }
   return ans;
}


// Von mises distribution
static double vonMises(double x, double mu, double kappa){
  return exp(kappa * cos(x - mu)) /*/ (2 * M_PI * bessj0(kappa)) */;
}

// Rotation utils
inline double mod2pi_positive(double vin)
{
    double q = vin / (2*M_PI) + 0.5;
    int qi = (int) q;

    return vin - qi*2*M_PI;
}

/** Map v to [-PI, PI] **/
inline double mod2pi(double vin)
{
    if (vin < 0)
        return -mod2pi_positive(-vin);
    else
        return mod2pi_positive(vin);
}

// static Eigen::Vector3d subtractQuats(const Eigen::Quaterniond & quat1, const Eigen::Quaterniond & quat2)
// {
//   Eigen::Quaterniond quat_resid = quat2.inverse() * quat1;
//   Eigen::AngleAxisd angle_axis_resid(quat_resid);

//   double angle = angle_axis_resid.angle();
//   angle = mod2pi(angle);
//   return angle_axis_resid.axis() * angle;
// }

// static Eigen::Vector3d getEulerAngles(const Eigen::Quaterniond & quat)
// {
//   return quat.toRotationMatrix().eulerAngles(2, 1, 0).reverse();
// }

// static Eigen::Vector3d getEulerAngles(const Eigen::Matrix3d & rot)
// {
//   return rot.eulerAngles(2, 1, 0).reverse(); //RPY
// }

}
