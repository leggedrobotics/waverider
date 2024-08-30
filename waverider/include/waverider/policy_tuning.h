#ifndef WAVERIDER_POLICY_TUNING_H_
#define WAVERIDER_POLICY_TUNING_H_

namespace waverider {
struct PolicyTuning {
  //! Overall activation of the policy
  float r = 1.3;
  //! Soft max tuning
  float c = 0.2;
  //! Repulsive gain > 0.0 (eq. symbol n)
  float eta_rep = 22;
  //! Positive length scale > 0.0 (eq. symbol v)
  float nu_rep = 1.4;
  //! Damping gain (eq. symbol n)
  float eta_damp = 140;
  //! Damping length scale > 0.0 (eq. symbol v)
  float nu_damp = 1.2;
  //! Whether to enable damping
  bool enable_damper = true;
  //! Whether to enable repulsion
  bool enable_repulsor = true;
};
}  // namespace waverider

#endif  // WAVERIDER_POLICY_TUNING_H_
