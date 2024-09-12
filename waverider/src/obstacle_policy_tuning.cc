#include "waverider/obstacle_policy_tuning.h"

namespace waverider {
DECLARE_CONFIG_MEMBERS(ObstaclePolicyTuning,
                      (r)
                      (c)
                      (eta_rep)
                      (nu_rep)
                      (eta_damp)
                      (nu_damp)
                      (enable_damper)
                      (enable_repulsor));

bool ObstaclePolicyTuning::isValid(bool verbose) const {
  bool is_valid = true;

  is_valid &= IS_PARAM_GT(r, 0.f, verbose);
  is_valid &= IS_PARAM_GT(c, 0.f, verbose);
  is_valid &= IS_PARAM_GT(eta_rep, 0.f, verbose);
  is_valid &= IS_PARAM_GT(nu_rep, 0.f, verbose);
  is_valid &= IS_PARAM_GT(nu_damp, 0.f, verbose);

  return is_valid;
}
}  // namespace waverider
