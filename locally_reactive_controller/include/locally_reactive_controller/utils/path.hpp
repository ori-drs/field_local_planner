#pragma once
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

class PathSegment
{
private:
  Eigen::Isometry3d p1_;
  Eigen::Isometry3d p2_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  PathSegment(const Eigen::Isometry3d& T1, const Eigen::Isometry3d& T2){
    p1_ = T1;
    p2_ = T2;
  }

  const Eigen::Isometry3d& p1() const { return p1_; }
  const Eigen::Isometry3d& p2() const { return p2_; }
};