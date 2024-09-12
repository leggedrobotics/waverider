#ifndef WAVERIDER_GOAL_POLICY_TUNING_H_
#define WAVERIDER_GOAL_POLICY_TUNING_H_

#include <wavemap/core/config/config_base.h>

namespace waverider {
struct GoalPolicyTuning : wavemap::ConfigBase<GoalPolicyTuning, 5> {
  //! Attractive gain
  float alpha = 20.f;
  //! Damping gain
  float beta = 25.f;
  //! Soft max tuning
  float c = 0.2;
  //! Metric scaling (applied as A = a * Identity)
  float a = 10.f;
  //! Disable attractor near boal
  bool disable_attractor_near_goal = true;

  static MemberMap memberMap;

  bool isValid(bool verbose) const override;
};
}  // namespace waverider

#endif  // WAVERIDER_GOAL_POLICY_TUNING_H_
