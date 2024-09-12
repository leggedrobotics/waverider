#include "waverider/waverider_policy.h"

#include <wavemap/core/utils/profiler_interface.h>

#include "waverider/obstacle_filter.h"

namespace waverider {
void WaveriderPolicy::updateObstacles(const wavemap::HashedWaveletOctree& map,
                                      const Point3D& robot_position,
                                      const Plane3D& ground_plane) {
  ProfilerZoneScoped;
  obstacle_filter_.update(map, robot_position, ground_plane);
}

rmpcpp::PolicyValue<3> WaveriderPolicy::evaluateAt(const rmpcpp::State<3>& x) {
  ProfilerZoneScoped;
  if (!isReady()) {
    return {Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero()};
  }

  const Eigen::Vector3f x_pos = x.pos_.cast<float>();
  const Eigen::Vector3f x_vel = x.vel_.cast<float>();

  std::vector<rmpcpp::PolicyValue<3>> all_values;
  const auto& policy_cells = obstacle_filter_.getObstacleCells();
  for (int i = 0; i < static_cast<int>(policy_cells.cell_widths.size()); i++) {
    if (i == 0 || run_all_levels_) {
      ParallelizedPolicy level_policy(tuning_);
      level_policy.setR(1.5f * WavemapObstacleFilter::maxRangeForHeight(i));

      auto level_value =
          level_policy.evaluate(policy_cells.centers[i], x_pos, x_vel);
      all_values.emplace_back(std::move(level_value));
    }
  }

  return rmpcpp::PolicyValue<3>::sum(all_values);
}
}  // namespace waverider
