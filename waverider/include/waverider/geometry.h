#ifndef WAVERIDER_GEOMETRY_H_
#define WAVERIDER_GEOMETRY_H_

#include <rmpcpp/geometry/partial_geometry.h>

namespace waverider {
class R3toR2 : public rmpcpp::PartialGeometry<3, 2> {
 public:
  R3toR2() : PartialGeometry<3, 2>(j_r3_to_r2()) {}

 private:
  static PartialGeometry<3, 2>::J_phi j_r3_to_r2() {
    PartialGeometry<3, 2>::J_phi jacobian;
    jacobian.setZero();
    jacobian.diagonal().head<2>() = Eigen::Vector2d::Ones();
    return jacobian;
  }
};

class R3toSE2 : public rmpcpp::PartialGeometry<3, 3> {
 public:
  R3toSE2() : PartialGeometry<3, 3>(j_r3_to_se2()) {}

 private:
  static PartialGeometry<3, 3>::J_phi j_r3_to_se2() {
    PartialGeometry<3, 3>::J_phi jacobian;
    jacobian.setZero();
    jacobian.diagonal().head<2>() = Eigen::Vector2d::Ones();
    return jacobian;
  }
};

inline rmpcpp::State<3> SE3toSE2(rmpcpp::SE3State se3_state) {
  const double yaw = se3_state.q().vec().z();
  const double yaw_dot = se3_state.w().z();
  rmpcpp::State<3>::Vector x{se3_state.pos_.x(), se3_state.pos_.y(), yaw};
  rmpcpp::State<3>::Vector x_dot{se3_state.vel_.x(), se3_state.vel_.y(),
                                 yaw_dot};
  return {x, x_dot};
}
}  // namespace waverider

#endif  // WAVERIDER_GEOMETRY_H_
