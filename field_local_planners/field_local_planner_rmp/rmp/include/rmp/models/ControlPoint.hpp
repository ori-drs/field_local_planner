/**
 *  @file  ControlPoint.h
 *  @author Matias Mattamala
 **/

#pragma once

#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/expressions.h>
#include <iostream>

/**
 * Control point2 maps a planar acceleration to a different frame
 */

namespace gtsam {

class GTSAM_EXPORT ControlPoint2 {
  Vector2 point_;    // Position in fixed frame
  Pose2 pose_;       // Pose of the control point
  Matrix23 J_;       // Full Jacobian from fixed frame to control point frame
  Matrix22 J_f_cp_;  // Jacobian from control point to fixed frame
  Matrix23 J_cp_b_;  // Jacobian from base to control point

 public:
  enum { dimension = 2 };
  typedef boost::shared_ptr<ControlPoint2> shared_ptr;  ///< shared pointer to object

  /*
   * Constructors
   */
  ControlPoint2(const Vector2& point) { ControlPoint2(Pose2(0, 0, 0), point); }

  ControlPoint2(const Pose2& T_a_b, const Vector2& p_b_cp) {
    // Pose that represents the control point in the fixed frame
    pose_ = T_a_b * Pose2(0.0, p_b_cp);

    // Create zero vector to obtain the position of the control point in the fixed frame and the jacobian
    Vector2 zero = Vector2::Zero();
    Matrix23 Jpose;
    point_ = pose_.transformFrom(zero, Jpose, J_f_cp_);

    J_cp_b_ << Matrix2::Identity(), Vector2(-p_b_cp.y(), p_b_cp.x());
    J_ = J_f_cp_ * J_cp_b_;
  }

  Vector2 apply(const Vector3& acceleration, OptionalJacobian<2, 2> Hmodel = boost::none, OptionalJacobian<2, 3> Hacc = boost::none) const {
    if (Hmodel) {
      *Hmodel = I_2x2;
    }
    if (Hacc) {
      *Hacc << J_;
    }

    // Transform acceleration
    Vector2 output = J_ * acceleration;
    // std::cout << "- control point -" << std::endl;
    // std::cout << J_ << std::endl;
    // std::cout << acceleration.transpose() << std::endl;
    // std::cout << output.transpose() << std::endl;
    // std::cout << "- -" << std::endl;

    return output;
  }

  Vector3 applyInverse(const Vector2 acceleration2) { return J_.transpose() * acceleration2; }

  Matrix2 applyMetric(const Matrix3& metric) const { return J_ * metric * J_.transpose(); }

  Matrix3 applyMetricInverse(const Matrix2& metric) const { return J_.transpose() * metric * J_; }

  Vector2 vector() const { return point_; }

  Vector2 point() const { return point_; }

  Pose2 pose() const { return pose_; }

  Matrix23 jacobian() const { return J_; }

  /// Output stream operator
  GTSAM_EXPORT friend std::ostream& operator<<(std::ostream& os, const ControlPoint2& cal);

  /// print with optional string
  void print(const std::string& s = "ControlPoint2") const;

  /// Check if equal up to specified tolerance
  bool equals(const ControlPoint2& model, double tol = 10e-9) const;

  /// @}
  /// @name Manifold
  /// @{

  /// return DOF, dimensionality of tangent space
  inline size_t dim() const { return dimension; }

  /// return DOF, dimensionality of tangent space
  static size_t Dim() { return dimension; }

  /// Given 5-dim tangent vector, create new Control point Model
  inline ControlPoint2 retract(const Vector& d) const { return ControlPoint2(point_ + d); }

  /// Unretraction for the model
  Vector2 localCoordinates(const ControlPoint2& T2) const { return T2.vector() - vector(); }
};

// Traits
template <>
struct traits<ControlPoint2> : public internal::Manifold<ControlPoint2> {};

template <>
struct traits<const ControlPoint2> : public internal::Manifold<ControlPoint2> {};

}  // namespace gtsam