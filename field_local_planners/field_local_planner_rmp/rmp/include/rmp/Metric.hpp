/**
 *  @file  Metric.hpp
 *  @brief Implements helper functions to make metrics
 *  @author Matias Mattamala
 **/

#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <boost/optional.hpp>
#include <iostream>

namespace gtsam {
namespace rmp {

template <typename TM = Vector2, typename METRIC = Matrix22>
class Metric {
 public:
  // Interface to make metrics
  static METRIC make(const std::string& type,        // Type of the metric
                     const double& offset = 0.0,     // offset used for logistic-like metrics
                     const double& steepness = 0.0,  // stepness for logistic-like metrics
                     const double& state = 0.0,      // 'state' - a value to evaluate the metric
                     const TM& twist = TM::Zero()   // velocity (if required)
  ) {
    // Check different types of metric
    if (type == "logistic") {
      return makeLogisticMetric(state, offset, steepness);

    } else if (type == "inv_logistic") {
      return makeInverseLogisticMetric(state, offset, steepness);

    } else if (type == "velocity") {
      return makeVelocityMetric(state, offset, steepness, twist);

    } else {
      // Constant metric
      return makeConstantMetric();
    }
  }

  // Explicit methods for each metric
  static METRIC makeLogisticMetric(const double& state, const double& offset, const double& steepness) {
    return logistic(state, offset, steepness) * METRIC::Identity();
  }

  static METRIC makeInverseLogisticMetric(const double& state, const double& offset, const double& steepness) {
    return inverseLogistic(state, offset, steepness) * METRIC::Identity();
  }

  static METRIC makeVelocityMetric(const double& state, const double& offset, const double& steepness, const TM& twist) {
    return inverseLogistic(state, offset, steepness) * METRIC(twist * twist.transpose());
  }

  static METRIC makeConstantMetric() { return METRIC::Identity(); }

  // helpers
  static double logistic(double x, double x0, double k) { return 1.0 / (1 + exp(-k * (x - x0))); }
  static double inverseLogistic(double x, double x0, double k) { return 1.0 / (1 + exp(k * (x - x0))); }
};

// Forward
using Metric2 = Metric<Vector2, Matrix22>;
using Metric3 = Metric<Vector3, Matrix33>;

}  // namespace rmp
}  // namespace gtsam