/**
 *  @file  Metric.hpp
 *  @brief Implements helper functions to make metrics
 *  @author Matias Mattamala
 **/

#pragma once

#include <rmp/models/AckermannModel.hpp>
#include <rmp/models/ControlPoint.hpp>
#include <rmp/models/DifferentialModel.hpp>

namespace gtsam {
namespace rmp {

// ControlPoint2
using ControlPoint2_ = Expression<ControlPoint2>;

template <class ACC>
inline Vector2 applyControlPoint2Helper(const ControlPoint2& control_point, const ACC& acceleration,
                                        OptionalJacobian<2, 2> Hmodel = boost::none, OptionalJacobian<2, 3> Hacc = boost::none) {
  return control_point.apply(acceleration, Hmodel, Hacc);
}

template <class ACC>
Vector2_ pullback(const Expression<ControlPoint2>& model_, const Expression<ACC>& acceleration_) {
  return Vector2_(applyControlPoint2Helper<ACC>, model_, acceleration_);
}

// AckermannModel
using AckermannModel_ = Expression<AckermannModel>;

template <class ACC>
inline Vector3 applyAckermannModelHelper(const AckermannModel& model, const ACC& acceleration, OptionalJacobian<3, 5> Hmodel = boost::none,
                                         OptionalJacobian<3, 2> Hacc = boost::none) {
  return model.apply(acceleration, Hmodel, Hacc);
}

template <class ACC>
Vector3_ pullback(const Expression<AckermannModel>& model_, const Expression<ACC>& acceleration_) {
  return Vector3_(applyAckermannModelHelper<ACC>, model_, acceleration_);
}

// DifferentialModel
using DifferentialModel_ = Expression<DifferentialModel>;

template <class ACC>
inline Vector3 applyDifferentialModelHelper(const DifferentialModel& model, const ACC& acceleration,
                                            OptionalJacobian<3, 1> Hmodel = boost::none, OptionalJacobian<3, 2> Hacc = boost::none) {
  return model.apply(acceleration, Hmodel, Hacc);
}

template <class ACC>
Vector3_ pullback(const Expression<DifferentialModel>& model_, const Expression<ACC>& acceleration_) {
  return Vector3_(applyDifferentialModelHelper<ACC>, model_, acceleration_);
}

}  // namespace rmp
}  // namespace gtsam