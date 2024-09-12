#include "waverider/goal_policy.h"

namespace waverider {
GoalPolicy::GoalPolicy(GoalPolicy::Vector target, GoalPolicy::Matrix A,
                       double alpha, double beta, double c)
    : alpha_(alpha), beta_(beta), c_(c), target_(std::move(target)) {
  this->A_static_ = std::move(A);
}

GoalPolicy::GoalPolicy(GoalPolicy::Vector target)
    : target_(std::move(target)) {}

GoalPolicy::GoalPolicy() : target_{Vector::Zero()} {}

void GoalPolicy::setTuning(double alpha, double beta, double gamma) {
  alpha_ = alpha;
  beta_ = beta;
  c_ = gamma;
}

GoalPolicy::PValue GoalPolicy::evaluateAt(const GoalPolicy::PState& state) {
  Vector f =
      alpha_ * s(this->space_.minus(target_, state.pos_)) - beta_ * state.vel_;
  return {f, this->A_static_};
}
}  // namespace waverider
