#include "waverider/ferrous_surface_policy_tuning.h"

namespace waverider {
DECLARE_CONFIG_MEMBERS(FerrousSurfacePolicyTuning,
                      (alpha)
                      (lambda)
                      (r)
                      (kappa)
                      (c)
                      (a));

bool FerrousSurfacePolicyTuning::isValid(bool verbose) const {
  bool is_valid = true;

  is_valid &= IS_PARAM_GT(alpha, 0.f, verbose);
  is_valid &= IS_PARAM_GT(lambda, 0.f, verbose);
  is_valid &= IS_PARAM_GT(r, 0.f, verbose);
  is_valid &= IS_PARAM_GT(kappa, 0.f, verbose);
  is_valid &= IS_PARAM_GT(c, 0.f, verbose);
  is_valid &= IS_PARAM_GT(a, 0.f, verbose);

  return is_valid;
}

}  // namespace waverider
