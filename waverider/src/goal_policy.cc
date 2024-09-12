#include "waverider/goal_policy.h"

namespace waverider {
GoalPolicy::GoalPolicy(GoalPolicy::Vector target, GoalPolicy::Matrix A,
                       double alpha, double beta, double c)
    : alpha_(alpha), beta_(beta), c_(c), target_(std::move(target)) {
  A_static_ = std::move(A);
}

GoalPolicy::GoalPolicy(GoalPolicy::Vector target)
    : target_(std::move(target)) {}

GoalPolicy::GoalPolicy() : target_{Vector::Zero()} {}

void GoalPolicy::setTuning(double alpha, double beta, double gamma,
                           bool disable_attractor_near_goal) {
  alpha_ = alpha;
  beta_ = beta;
  c_ = gamma;
  disable_attractor_near_goal_ = disable_attractor_near_goal;
}

GoalPolicy::PValue GoalPolicy::evaluateAt(const GoalPolicy::PState& state) {
  const auto error = space_.minus(target_, state.pos_);
  double alpha_scaled = alpha_;
  if (disable_attractor_near_goal_) {
    alpha_scaled *= std::min(error.norm(), 1.0);
  }
  Vector f = alpha_scaled * s(error) - beta_ * state.vel_;
  return {f, A_static_};
}
}  // namespace waverider
