#ifndef WAVERIDER_GEOMETRY_H_
#define WAVERIDER_GEOMETRY_H_

#include <rmpcpp/geometry/partial_geometry.h>

namespace waverider {
class R3toR2 : public rmpcpp::PartialGeometry<3, 2> {
 public:
  R3toR2() : PartialGeometry<3, 2>(j_r3_to_r2()) {}

 private:
  static J_phi j_r3_to_r2() {
    J_phi jacobian;
    jacobian.setZero();
    jacobian.diagonal().head<2>() = Eigen::Vector2d::Ones();
    return jacobian;
  }
};

class R3toSE2 : public rmpcpp::PartialGeometry<3, 3> {
 public:
  R3toSE2() : PartialGeometry<3, 3>(j_r3_to_se2()) {}

 private:
  static J_phi j_r3_to_se2() {
    J_phi jacobian;
    jacobian.setZero();
    jacobian.diagonal().head<2>() = Eigen::Vector2d::Ones();
    return jacobian;
  }
};

class SE2toSE2Translated : public rmpcpp::GeometryBase<3, 3> {
 public:
  explicit SE2toSE2Translated(FloatingPoint x_offset) : x_offset(x_offset) {}

  J_phi J(const StateX& state) const override {
    J_phi jacobian;
    jacobian.setIdentity();
    const auto yaw = state.pos_[2];
    jacobian(0, 2) = x_offset * std::cos(yaw);
    jacobian(1, 2) = -x_offset * std::sin(yaw);
    return jacobian;
  }

  StateX convertToX(const StateQ& state_q) const override {
    throw std::logic_error("Not implemented");
    return StateX{};
  }

  StateQ convertToQ(const StateX& state_x) const override {
    throw std::logic_error("Not implemented");
    return StateQ{};
  }

 private:
  const FloatingPoint x_offset;
};

inline rmpcpp::State<3> SE3toSE2(rmpcpp::SE3State se3_state) {
  const auto rotmat = se3_state.q().toRotationMatrix().block<2, 2>(0, 0);
  const double yaw = std::atan2(rotmat(1, 0), rotmat(0, 0));
  const double yaw_dot = se3_state.w().z();
  rmpcpp::State<3>::Vector x{se3_state.pos_.x(), se3_state.pos_.y(), yaw};
  rmpcpp::State<3>::Vector x_dot{se3_state.vel_.x(), se3_state.vel_.y(),
                                 yaw_dot};
  return {x, x_dot};
}
}  // namespace waverider

#endif  // WAVERIDER_GEOMETRY_H_
