#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>

namespace field_local_planner {

using Twist = gtsam::Vector6;
using Path = std::vector<gtsam::Pose3>;
using Time = uint64_t;

}  // namespace field_local_planner