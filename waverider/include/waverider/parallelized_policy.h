#ifndef WAVERIDER_PARALLELIZED_POLICY_H_
#define WAVERIDER_PARALLELIZED_POLICY_H_

#include <vector>

#include <rmpcpp/core/policy_value.h>

#include "waverider/obstacle_filter.h"
#include "waverider/obstacle_policy_tuning.h"

namespace waverider {
class ParallelizedPolicy {
 public:
  explicit ParallelizedPolicy(ObstaclePolicyTuning tuning);

  rmpcpp::PolicyValue<3> evaluate(
      const std::vector<Eigen::Vector3f>& x_observations,
      const Eigen::Vector3f& x, const Eigen::Vector3f& xdot) const;

  void setR(float r) {
    tuning_.r = r;
    tuning_.nu_rep = 0.5f * r;
    tuning_.nu_damp = 0.3f * r;
  }

 private:
  ObstaclePolicyTuning tuning_;

  Eigen::Vector3f s(const Eigen::Vector3f& x) const { return x / h(x.norm()); }

  // Softmax helper function
  float h(float z) const {
    return (z + tuning_.c * std::log(1.f + std::exp(-2.f * tuning_.c * z)));
  }
  static float wr(float s, float r) {
    if (s > r) {
      return 0;
    }
    const float c2 = 1.f / (r * r);
    const float c1 = -2.f / r;
    return (c2 * s * s) + (c1 * s) + 1.f;
  }
};
}  // namespace waverider

#endif  // WAVERIDER_PARALLELIZED_POLICY_H_
