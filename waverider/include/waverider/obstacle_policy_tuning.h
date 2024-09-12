#ifndef WAVERIDER_OBSTACLE_POLICY_TUNING_H_
#define WAVERIDER_OBSTACLE_POLICY_TUNING_H_

#include <wavemap/core/config/config_base.h>

namespace waverider {
struct ObstaclePolicyTuning : wavemap::ConfigBase<ObstaclePolicyTuning, 8> {
  //! Overall activation of the policy
  //! @warning Gets set automatically. Manual values ignored.
  float r = 1.3;
  //! Soft max tuning
  float c = 0.2;
  //! Repulsive gain > 0.0 (eq. symbol n)
  float eta_rep = 22.0;
  //! Positive length scale > 0.0 (eq. symbol v)
  //! @warning Gets set automatically. Manual values ignored.
  float nu_rep = 1.4;
  //! Damping gain (eq. symbol n)
  float eta_damp = 140.0;
  //! Damping length scale > 0.0 (eq. symbol v)
  //! @warning Gets set automatically. Manual values ignored.
  float nu_damp = 1.2;
  //! Whether to enable damping
  bool enable_damper = true;
  //! Whether to enable repulsion
  bool enable_repulsor = true;

  static MemberMap memberMap;

  bool isValid(bool verbose) const override;
};
}  // namespace waverider

#endif  // WAVERIDER_OBSTACLE_POLICY_TUNING_H_
