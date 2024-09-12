#include "waverider/goal_policy.h"

namespace waverider {
GoalPolicy::GoalPolicy(const GoalPolicyTuning& tuning) { setTuning(tuning); }

void GoalPolicy::setTuning(const GoalPolicyTuning& tuning) {
  tuning_ = tuning.checkValid();
  A_static_ = tuning_.a * Matrix::Identity();
}

GoalPolicy::PValue GoalPolicy::evaluateAt(const GoalPolicy::PState& state) {
  const auto error = space_.minus(target_, state.pos_);
  double alpha_scaled = tuning_.alpha;
  if (tuning_.disable_attractor_near_goal) {
    alpha_scaled *= std::min(error.norm(), 1.0);
  }
  Vector f = alpha_scaled * s(error) - tuning_.beta * state.vel_;
  return {f, A_static_};
}
}  // namespace waverider
