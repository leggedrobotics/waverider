#ifndef WAVERIDER_YAW_POLICY_H_
#define WAVERIDER_YAW_POLICY_H_

#include <algorithm>

#include <waverider/yaw_policy_tuning.h>

#include "rmpcpp/core/policy_base.h"

namespace waverider {
class YawPolicy : public rmpcpp::PolicyBase<rmpcpp::Space<3>> {
 public:
  explicit YawPolicy(const YawPolicyTuning& tuning);
  YawPolicy() = default;

  void setTuning(const YawPolicyTuning& tuning);

  void setTarget(const Vector& target) { target_ = target; }

  Vector& target() { return target_; }

  PValue evaluateAt(const PState& state) override;

 protected:
  YawPolicyTuning tuning_;
  Vector target_ = Vector::Zero();

  /**
   *  Normalization helper function.
   */
  Vector s(const Vector& x) { return x / h(space_.norm(x)); }

  /**
   * Softmax helper function
   */
  double h(const double z) const {
    return z + tuning_.c * std::log(1.0 + exp(-2.0 * tuning_.c * z));
  }

  static double wrapAngle(double x) {
    x = std::fmod(x + M_PI, 2.0 * M_PI);
    if (x < 0.0) x += 2.0 * M_PI;
    return x - M_PI;
  }
};
}  // namespace waverider

#endif  // WAVERIDER_YAW_POLICY_H_
