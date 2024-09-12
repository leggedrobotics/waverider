#ifndef WAVERIDER_WAVERIDER_POLICY_H_
#define WAVERIDER_WAVERIDER_POLICY_H_

#include <utility>

#include <rmpcpp/core/policy_base.h>
#include <rmpcpp/core/state.h>
#include <wavemap/core/map/hashed_wavelet_octree.h>

#include "waverider/parallelized_policy.h"

namespace waverider {
class WaveriderPolicy : public rmpcpp::PolicyBase<rmpcpp::Space<3>> {
 public:
  WaveriderPolicy() = default;

  bool isReady() const { return obstacle_filter_.isReady(); }

  void setTuning(const ObstaclePolicyTuning& tuning);
  void setOccupancyThreshold(FloatingPoint value) {
    obstacle_filter_.setOccupancyThreshold(value);
  }
  void setUseMultiResolution(bool use_multi_resolution) {
    use_multi_resolution_ = use_multi_resolution;
  }

  void updateObstacles(const wavemap::HashedWaveletOctree& map,
                       const Point3D& robot_position,
                       const Plane3D& ground_plane);
  const ObstacleCells& getObstacleCells() {
    return obstacle_filter_.getObstacleCells();
  }

  rmpcpp::PolicyValue<3> evaluateAt(const rmpcpp::State<3>& x) override;

 public:
  WavemapObstacleFilter obstacle_filter_;
  bool use_multi_resolution_ = true;
  ObstaclePolicyTuning tuning_;
};
}  // namespace waverider

#endif  // WAVERIDER_WAVERIDER_POLICY_H_
