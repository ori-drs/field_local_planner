#include <rmp/models/ControlPoint.hpp>

#include <cmath>
#include <fstream>
#include <iostream>

namespace gtsam {

std::ostream& operator<<(std::ostream& os, const ControlPoint2& model) {
  os << "x: " << model.point()(0) << ", y: " << model.point()(1);
  return os;
}

/* ************************************************************************* */
void ControlPoint2::print(const std::string& s) const {
  gtsam::print((Vector)vector(), s);
}

/* ************************************************************************* */
bool ControlPoint2::equals(const ControlPoint2& model, double tol) const {
  return (std::fabs(point_(0) - model.point_(0)) < tol && std::fabs(point_(1) - model.point_(1)) < tol);
}

}  // namespace gtsam