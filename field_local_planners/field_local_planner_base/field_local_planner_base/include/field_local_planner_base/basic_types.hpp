#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace field_local_planner {

using Twist = gtsam::Vector6;
using Path = std::vector<gtsam::Pose3>;
using Time = uint64_t;
using PointType = pcl::PointXYZI;

}  // namespace field_local_planner