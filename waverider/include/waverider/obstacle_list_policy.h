#ifndef WAVERIDER_OBSTACLE_LIST_POLICY_H_
#define WAVERIDER_OBSTACLE_LIST_POLICY_H_

#include <utility>
#include <vector>

#include <rmpcpp/core/policy_base.h>
#include <rmpcpp/core/state.h>
#include <wavemap/core/map/hashed_wavelet_octree.h>

#include "waverider/parallelized_policy.h"

namespace waverider {
class ObstacleListPolicy : public rmpcpp::PolicyBase<rmpcpp::Space<3>> {
 public:
  using ObstacleList = std::vector<wavemap::AABB<Point3D>>;

  ObstacleListPolicy() = default;

  void addObstacles(const ObstacleList& obstacles) {
    obstacle_aabbs_.insert(obstacle_aabbs_.end(), obstacles.begin(),
                           obstacles.end());
  }
  void clearObstacles() { obstacle_aabbs_.clear(); }

  void setTuning(ObstaclePolicyTuning tuning) { tuning_ = std::move(tuning); }

  rmpcpp::PolicyValue<3> evaluateAt(const rmpcpp::State<3>& x) override;

 public:
  ObstacleList obstacle_aabbs_;
  ObstaclePolicyTuning tuning_;
};
}  // namespace waverider

#endif  // WAVERIDER_OBSTACLE_LIST_POLICY_H_
