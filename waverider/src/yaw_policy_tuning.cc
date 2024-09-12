#include "waverider/yaw_policy_tuning.h"

namespace waverider {
DECLARE_CONFIG_MEMBERS(YawPolicyTuning,
                      (alpha)
                      (beta)
                      (c)
                      (a)
                      (angle_forward));

bool YawPolicyTuning::isValid(bool verbose) const {
  bool is_valid = true;

  is_valid &= IS_PARAM_GT(alpha, 0.f, verbose);
  is_valid &= IS_PARAM_GT(c, 0.f, verbose);
  is_valid &= IS_PARAM_GT(a, 0.f, verbose);

  return is_valid;
}
}  // namespace waverider
