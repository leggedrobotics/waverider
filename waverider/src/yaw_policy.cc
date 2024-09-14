#include "waverider/yaw_policy.h"

namespace waverider {
YawPolicy::YawPolicy(const YawPolicyTuning& tuning) { setTuning(tuning); }

void YawPolicy::setTuning(const YawPolicyTuning& tuning) {
  tuning_ = tuning.checkValid();
  A_static_.setZero();
  A_static_(2, 2) = tuning_.a;
}

YawPolicy::PValue YawPolicy::evaluateAt(const YawPolicy::PState& state) {
  const double speed = state.vel_.head<2>().norm();
  const double alpha_scaled = tuning_.alpha * std::min(speed, 1.0);
  const double yaw = state.pos_[2];
  // const double yaw_target =
  //     std::atan2(state.vel_.y(), state.vel_.x()) + tuning_.angle_forward;
  const double yaw_target =
      std::atan2(target_.y() - state.pos_[1], target_.x() - state.pos_[0]) +
      tuning_.angle_forward;
  const double yaw_error = wrapAngle(yaw_target - yaw);
  const Eigen::Vector3d error{0.0, 0.0, yaw_error};
  const Eigen::Vector3d v{0.0, 0.0, state.vel_[2]};
  Vector f = alpha_scaled * s(error) - tuning_.beta * v;
  return {f, A_static_};
}
}  // namespace waverider
