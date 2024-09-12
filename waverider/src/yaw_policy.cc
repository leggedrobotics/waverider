#include "waverider/yaw_policy.h"

namespace waverider {
YawPolicy::YawPolicy(YawPolicy::Matrix A, double alpha, double beta, double c)
    : alpha_(alpha), beta_(beta), c_(c) {
  A_static_ = std::move(A);
}

void YawPolicy::setTuning(double alpha, double beta, double gamma) {
  alpha_ = alpha;
  beta_ = beta;
  c_ = gamma;
}

YawPolicy::PValue YawPolicy::evaluateAt(const YawPolicy::PState& state) {
  const double speed = state.vel_.head<2>().norm();
  const double alpha_scaled = alpha_ * std::min(speed, 1.0);
  const double yaw = state.pos_[2];
  const double yaw_target = std::atan2(state.vel_.y(), state.vel_.x());
  const double yaw_error = wrapAngle(yaw_target - yaw);
  const Eigen::Vector3d error{0.0, 0.0, yaw_error};
  const Eigen::Vector3d v{0.0, 0.0, state.vel_[2]};
  Vector f = alpha_scaled * s(error) - beta_ * v;
  return {f, A_static_};
}
}  // namespace waverider
