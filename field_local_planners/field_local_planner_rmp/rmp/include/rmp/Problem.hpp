/**
 *  @file  RmpProblem.h
 *  @brief Wrapper of a ExpresisonFactorGraph to define a RMP problem
 *  @author Matias Mattamala
 */

#pragma once

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/slam/expressions.h>
#include "RiemannianMotionPolicy.hpp"

namespace gtsam {
namespace rmp {

/**
 * A RMP problem is defined as wrapper to an expression factor
 */
class Problem : public ExpressionFactorGraph {
 public:
  /**
   * RMP problem parameters
   */
  struct Parameters {
    std::string ordering_;       // COLAMD, METIS, NATURAL, CUSTOM
    std::string verbosity_;      // SILENT, TERMINATION, ERROR, VALUES, DELTA, LINEAR
    std::string linear_solver_;  // MULTIFRONTAL_CHOLESKY, MULTIFRONTAL_QR, SEQUENTIAL_CHOLESKY, SEQUENTIAL_QR,
                                 // Iterative (Experimental), CHOLMOD (Experimental)
    std::string algorithm_;      // GAUSS_NEWTON, LM, DOGLEG

    Parameters() : ordering_("COLAMD"), verbosity_("SILENT"), linear_solver_("MULTIFRONTAL_CHOLESKY"), algorithm_("GAUSS_NEWTON"){};
  };

 private:
  Values solution_;  // Acceleration solution
  Matrix metric_;    // Output metric

 public:
  /**
   * ExpressionFactor implements |h(x)-z|^2_R directly from the variable (h(x)) and the RMP (z) using the metric as noise model
   */
  template <typename ACC, typename METRIC>
  void addRmp(const Expression<ACC>& variable, const RiemannianMotionPolicy<ACC, METRIC>& rmp) {
    addExpressionFactor(variable, rmp.acceleration(), noiseModel::Gaussian::Information(rmp.metric()));
  }

  template <typename ACC, typename METRIC>
  void addRmp(const Expression<ACC>& variable, const ACC& acc, const METRIC& metric) {
    addExpressionFactor(variable, acc, noiseModel::Gaussian::Information(metric));
  }

  /**
   * Get solution
   */
  Values solution() const { return solution_; }
  Matrix metric() const { return metric_; }

  /**
   * Instantiates the optimizer and returns the optimization solution
   */
  Values solve(const Values& initial_values, bool iterate = false, const Parameters& params = Parameters()) {
    // Create optimizer
    std::shared_ptr<NonlinearOptimizer> optimizer;

    if (params.algorithm_ == "GAUSS_NEWTON") {
      GaussNewtonParams parameters;
      parameters.setOrderingType(params.ordering_);
      parameters.setVerbosity(params.verbosity_);
      parameters.setLinearSolverType(params.linear_solver_);
      optimizer = std::make_shared<GaussNewtonOptimizer>(*this, initial_values, parameters);

    } else if (params.algorithm_ == "LM") {
      LevenbergMarquardtParams parameters;
      parameters.setOrderingType(params.ordering_);
      parameters.setVerbosity(params.verbosity_);
      parameters.setLinearSolverType(params.linear_solver_);
      optimizer = std::make_shared<LevenbergMarquardtOptimizer>(*this, initial_values, parameters);

    } else if (params.algorithm_ == "DOGLEG") {
      DoglegParams parameters;
      parameters.setOrderingType(params.ordering_);
      parameters.setVerbosity(params.verbosity_);
      parameters.setLinearSolverType(params.linear_solver_);
      optimizer = std::make_shared<DoglegOptimizer>(*this, initial_values, parameters);
    } else {
      throw std::invalid_argument("Invalid optimization algorithm [" + params.algorithm_ + "]");
    }

    // Solve
    if (!iterate) {
      // RMP formulation assumes everything is linear, hence 1 iteration of
      // Gauss-Newton should find the solution
      optimizer->iterate();
      solution_ = optimizer->values();
    } else {
      // Otherwise, we should iterate to obtain the optimal acceleration
      solution_ = optimizer->optimize();
    }

    // Recover optimized metric
    Marginals marginals(ExpressionFactorGraph(*this), solution_);
    metric_ = marginals.marginalInformation(this->keyVector().at(0));

    return solution_;
  }
};

}  // namespace rmp
}  // namespace gtsam
