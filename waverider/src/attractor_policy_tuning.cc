#include "waverider/attractor_policy_tuning.h"

namespace waverider {
DECLARE_CONFIG_MEMBERS(AttractorPolicyTuning,
                      (alpha)
                      (beta)
                      (c)
                      (a));

bool AttractorPolicyTuning::isValid(bool verbose) const {
  bool is_valid = true;

  is_valid &= IS_PARAM_GT(alpha, 0.f, verbose);
  is_valid &= IS_PARAM_GT(c, 0.f, verbose);
  is_valid &= IS_PARAM_GT(a, 0.f, verbose);

  return is_valid;
}
}  // namespace waverider
