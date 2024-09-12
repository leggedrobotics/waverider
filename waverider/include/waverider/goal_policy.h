#ifndef WAVERIDER_GOAL_POLICY_H_
#define WAVERIDER_GOAL_POLICY_H_

#include "rmpcpp/core/policy_base.h"
#include "waverider/goal_policy_tuning.h"

namespace waverider {
class GoalPolicy : public rmpcpp::PolicyBase<rmpcpp::Space<3>> {
 public:
  /**
   * Sets up the policy.
   * target is the target to move to.
   * A is the metric to be used.
   * alpha, beta and c are tuning parameters.
   */
  explicit GoalPolicy(const GoalPolicyTuning& tuning);
  GoalPolicy() = default;

  void setTuning(const GoalPolicyTuning& tuning);

  void setTarget(const Vector& target) { target_ = target; }

  Vector& target() { return target_; }

  PValue evaluateAt(const PState& state) override;

 protected:
  GoalPolicyTuning tuning_;
  Vector target_ = Vector::Zero();

  /**
   *  Normalization helper function.
   */
  Vector s(const Vector& x) { return x / h(this->space_.norm(x)); }

  /**
   * Softmax helper function
   */
  double h(const double z) const {
    return (z + tuning_.c * std::log(1.0 + std::exp(-2.0 * tuning_.c * z)));
  }
};
}  // namespace waverider

#endif  // WAVERIDER_GOAL_POLICY_H_
