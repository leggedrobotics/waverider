#ifndef WAVERIDER_YAW_POLICY_H_
#define WAVERIDER_YAW_POLICY_H_

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
    const double yaw_current = state.pos_[2];
    const double yaw_desired = std::atan2(state.vel_.y(), state.vel_.x());
    const double yaw_error = wrapAngle(yaw_desired - yaw_current);
    const Eigen::Vector3d error{0.0, 0.0, yaw_error};
    const Eigen::Vector3d vel{0.0, 0.0, state.vel_[2]};
    Vector f = alpha_ * s(error) - beta_ * vel;
    return {f, A_static_};
  }

 protected:
  /**
   *  Normalization helper function.
   */
  inline Vector s(Vector x) { return x / h(space_.norm(x)); }

  /**
   * Softmax helper function
   */
  inline double h(const double z) const {
    return (z + c_ * log(1 + exp(-2 * c_ * z)));
  }

  inline double wrapAngle(double x) const {
    x = std::fmod(x + M_PI, 2.0 * M_PI);
    if (x < 0.0) x += 2.0 * M_PI;
    return x - M_PI;
  }

  double alpha_{1.0}, beta_{8.0}, c_{0.005};
};
}  // namespace waverider

#endif  // WAVERIDER_YAW_POLICY_H_
