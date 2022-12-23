#pragma once
// #include <ros/ros.h>
// #include <ros/console.h>
// #include <tf/tf.h>
// #include <eigen_conversions/eigen_msg.h>
// #include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <yaml-cpp/yaml.h>
#include <iostream>

namespace field_local_planner {
namespace utils {

#define COL_WIDTH 40

static inline std::string capitalize(const std::string& s) {
  // From https://stackoverflow.com/a/14494432
  bool cap = true;

  std::string capitalized = s;

  for (unsigned int i = 0; i <= s.length(); i++) {
    if (isalpha(capitalized[i]) && cap == true) {
      capitalized[i] = toupper(capitalized[i]);
      cap = false;
    } else if (isspace(capitalized[i])) {
      cap = true;
    }
  }

  return capitalized;
}

/// @brief Wrapper for yaml-cpp to provide default config files if not provided.
/// @author David Wisth
template <typename T>
T get(const YAML::Node& node, const int& param, const T& default_value, bool silent = false) {
  if (!node[param]) {
    // if(!silent) std::cout << std::left << std::setw(COL_WIDTH) << param << ": Not found. Using default value: " << default_value <<
    // std::endl;
    return default_value;
  }
  // if(!silent) std::cout << std::left << std::setw(COL_WIDTH) << ": " << node[param].as<T>() << std::endl;
  return node[param].as<T>();
}

template <typename T>
T get(const YAML::Node& node, const std::string& param, const T& default_value, bool silent = false) {
  if (!node[param]) {
    // if(!silent) std::cout << std::left << std::setw(COL_WIDTH) << param << ": Not found. Using default value: " << default_value <<
    // std::endl;
    return default_value;
  }
  // if(!silent) std::cout << std::left << std::setw(COL_WIDTH)  << param << ": " << node[param].as<T>() << std::endl;
  return node[param].as<T>();
}

static inline double clipValue(double value, double max_value) {
  return std::max(std::min(value, max_value), -max_value);
}

// constrain angle to be -180:180 in radians
static inline double constrainAngle(double x) {
  x = fmod(x + M_PI, 2.0 * M_PI);
  if (x < 0) x += 2.0 * M_PI;
  return x - M_PI;
}

/**
 * Convert the angle given in radians to degrees.
 * https://techoverflow.net/2019/05/30/c-equivalent-of-numpy-php-rad2deg/
 */
template <typename T>
static T rad2deg(T angle) {
  return angle * 180.0 / M_PI;
}
template <typename T>
static T deg2rad(T angle) {
  return angle / 180.0 * M_PI;
}

// Bessel function 1st kind, order 0
// from https://www.atnf.csiro.au/computing/software/gipsy/sub/bessel.c
static double bessj0(double x) {
  double ax, z;
  double xx, y, ans, ans1, ans2;

  if ((ax = fabs(x)) < 8.0) {
    y = x * x;
    ans1 = 57568490574.0 + y * (-13362590354.0 + y * (651619640.7 + y * (-11214424.18 + y * (77392.33017 + y * (-184.9052456)))));
    ans2 = 57568490411.0 + y * (1029532985.0 + y * (9494680.718 + y * (59272.64853 + y * (267.8532712 + y * 1.0))));
    ans = ans1 / ans2;
  } else {
    z = 8.0 / ax;
    y = z * z;
    xx = ax - 0.785398164;
    ans1 = 1.0 + y * (-0.1098628627e-2 + y * (0.2734510407e-4 + y * (-0.2073370639e-5 + y * 0.2093887211e-6)));
    ans2 = -0.1562499995e-1 + y * (0.1430488765e-3 + y * (-0.6911147651e-5 + y * (0.7621095161e-6 - y * 0.934935152e-7)));
    ans = sqrt(0.636619772 / ax) * (cos(xx) * ans1 - z * sin(xx) * ans2);
  }
  return ans;
}

// Von mises distribution
static double vonMises(double x, double mu, double kappa) {
  return exp(kappa * cos(x - mu)) /*/ (2 * M_PI * bessj0(kappa)) */;
}

// Rotation utils
inline double mod2pi_positive(double vin) {
  double q = vin / (2 * M_PI) + 0.5;
  int qi = (int)q;

  return vin - qi * 2 * M_PI;
}

/** Map v to [-PI, PI] **/
inline double mod2pi(double vin) {
  if (vin < 0)
    return -mod2pi_positive(-vin);
  else
    return mod2pi_positive(vin);
}

}  // namespace utils
}  // namespace field_local_planner