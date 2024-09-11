#ifndef WAVERIDER_YAW_POLICY_H_
#define WAVERIDER_YAW_POLICY_H_

#include <algorithm>

#include "rmpcpp/core/policy_base.h"

namespace waverider {
class YawPolicy : public rmpcpp::PolicyBase<rmpcpp::Space<3>> {
 public:
  YawPolicy(Matrix A, double alpha, double beta, double c)
      : alpha_(alpha), beta_(beta), c_(c) {
    A_static_ = A;
  }

  YawPolicy() = default;

  void setTuning(double alpha, double beta, double gamma) {
    alpha_ = alpha;
    beta_ = beta;
    c_ = gamma;
  }

  virtual PValue evaluateAt(const PState& state) {
    const double speed = state.vel_.head<2>().norm();
    const double alpha_scaled = alpha_ * std::min(speed, 1.0);
    const double yaw = state.pos_[2];
    const double yaw_target = std::atan2(state.vel_.y(), state.vel_.x());
    const double yaw_error = wrapAngle(yaw_target - yaw);
    const Eigen::Vector3d error{0.0, 0.0, yaw_error};
    const Eigen::Vector3d v{0.0, 0.0, state.vel_[2]};
    Vector f = alpha_scaled * s(error) - beta_ * v;
    return {f, A_static_};
  }

 protected:
  /**
   *  Normalization helper function.
   */
  Vector s(Vector x) { return x / h(space_.norm(x)); }

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
