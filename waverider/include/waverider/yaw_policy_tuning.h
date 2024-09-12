#ifndef WAVERIDER_YAW_POLICY_TUNING_H_
#define WAVERIDER_YAW_POLICY_TUNING_H_

#include <wavemap/core/config/config_base.h>

namespace waverider {
struct YawPolicyTuning : wavemap::ConfigBase<YawPolicyTuning, 5> {
  //! Attractive gain
  float alpha = 20.f;
  //! Damping gain
  float beta = 25.f;
  //! Soft max tuning
  float c = 0.2;
  //! Metric scaling (applied as A(i,j) = O except A(2,2) = a)
  float a = 10.f;
  //! Angle that is considered as 'facing forward'
  wavemap::ValueWithUnit<wavemap::SiUnit::kRadians, float> angle_forward = 0.f;

  static MemberMap memberMap;

  bool isValid(bool verbose) const override;
};
}  // namespace waverider

#endif  // WAVERIDER_YAW_POLICY_TUNING_H_
