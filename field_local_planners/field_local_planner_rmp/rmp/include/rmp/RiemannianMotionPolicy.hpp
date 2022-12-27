/**
 *  @file  RiemannianMotionPolicy.h
 *  @author Matias Mattamala
 **/

#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/linear/NoiseModel.h>
#include <iostream>

using namespace gtsam;

namespace gtsam {
namespace rmp {

template <typename M, typename TM, typename METRIC>
class RiemannianMotionPolicy {
 protected:
  using State = M;
  using Velocity = TM;
  using Acceleration = TM;
  using Metric = METRIC;

  // Internal variables
  Acceleration acceleration_;
  Metric metric_;

 public:
  RiemannianMotionPolicy() : acceleration_(Acceleration::Zero()), metric_(Metric::Identity()) {}

  RiemannianMotionPolicy(const Acceleration& acc, const Metric& metric) : acceleration_(acc), metric_(metric) {}

  const Acceleration acceleration() const { return acceleration_; }
  const Metric metric() const { return metric_; }
};

using Rmp1 = RiemannianMotionPolicy<Vector1, Vector1, Matrix1>;
using Rmp2 = RiemannianMotionPolicy<Vector2, Vector2, Matrix2>;
using Rmp3 = RiemannianMotionPolicy<Vector3, Vector3, Matrix3>;

}  // namespace rmp
}  // namespace gtsam