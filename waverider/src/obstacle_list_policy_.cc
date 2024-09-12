#include <wavemap/core/utils/profiler_interface.h>

#include "waverider/obstacle_list_policy.h"

namespace waverider {
ObstacleListPolicy::ObstacleListPolicy(const ObstaclePolicyTuning& tuning) {
  setTuning(tuning);
}

void ObstacleListPolicy::setTuning(const ObstaclePolicyTuning& tuning) {
  tuning_ = tuning.checkValid();
}

void ObstacleListPolicy::addObstacles(
    const ObstacleListPolicy::ObstacleList& obstacles) {
  obstacle_aabbs_.insert(obstacle_aabbs_.end(), obstacles.begin(),
                         obstacles.end());
}

rmpcpp::PolicyValue<3> ObstacleListPolicy::evaluateAt(
    const rmpcpp::State<3>& x) {
  ProfilerZoneScoped;
  if (obstacle_aabbs_.empty()) {
    return {Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero()};
  }

  const Eigen::Vector3f x_pos = x.pos_.cast<float>();
  const Eigen::Vector3f x_vel = x.vel_.cast<float>();

  std::vector<Point3D> closest_points;
  closest_points.reserve(obstacle_aabbs_.size());
  for (const auto& obstacle_aabb : obstacle_aabbs_) {
    closest_points.emplace_back(obstacle_aabb.closestPointTo(x_pos));
  }

  ParallelizedPolicy policy(tuning_);
  return policy.evaluate(closest_points, x_pos, x_vel);
}
}  // namespace waverider
