#ifndef WAVERIDER_YAW_POLICY_H_
#define WAVERIDER_YAW_POLICY_H_

#include <algorithm>

#include "rmpcpp/core/policy_base.h"

namespace waverider {
class YawPolicy : public rmpcpp::PolicyBase<rmpcpp::Space<3>> {
 public:
  YawPolicy(Matrix A, double alpha, double beta, double c);

  YawPolicy() = default;

  void setTuning(double alpha, double beta, double gamma);

  PValue evaluateAt(const PState& state) override;

 protected:
  /**
   *  Normalization helper function.
   */
  Vector s(const Vector& x) { return x / h(space_.norm(x)); }

  /**
   * Softmax helper function
   */
  double h(const double z) const {
    return z + c_ * std::log(1.0 + exp(-2.0 * c_ * z));
  }

  static double wrapAngle(double x) {
    x = std::fmod(x + M_PI, 2.0 * M_PI);
    if (x < 0.0) x += 2.0 * M_PI;
    return x - M_PI;
  }

  double alpha_{1.0}, beta_{8.0}, c_{0.005};
};
}  // namespace waverider

#endif  // WAVERIDER_YAW_POLICY_H_
