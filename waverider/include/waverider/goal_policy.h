#ifndef WAVERIDER_GOAL_POLICY_H_
#define WAVERIDER_GOAL_POLICY_H_

#include "rmpcpp/core/policy_base.h"

namespace waverider {
class GoalPolicy : public rmpcpp::PolicyBase<rmpcpp::Space<3>> {
 public:
  /**
   * Sets up the policy.
   * target is the target to move to.
   * A is the metric to be used.
   * alpha, beta and c are tuning parameters.
   */
  GoalPolicy(Vector target, Matrix A, double alpha, double beta, double c);
  explicit GoalPolicy(Vector target);
  GoalPolicy();

  void setTuning(double alpha, double beta, double gamma,
                 bool disable_attractor_near_goal);

  void setTarget(const Vector& target) { target_ = target; }

  Vector& target() { return target_; }

  PValue evaluateAt(const PState& state) override;

 protected:
  /**
   *  Normalization helper function.
   */
  Vector s(const Vector& x) { return x / h(this->space_.norm(x)); }

  /**
   * Softmax helper function
   */
  double h(const double z) const {
    return (z + c_ * std::log(1.0 + std::exp(-2.0 * c_ * z)));
  }

  double alpha_{1.0}, beta_{8.0}, c_{0.005};
  Vector target_;
  bool disable_attractor_near_goal_ = true;
};
}  // namespace waverider

#endif  // WAVERIDER_GOAL_POLICY_H_
