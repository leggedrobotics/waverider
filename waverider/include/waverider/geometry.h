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
}  // namespace waverider

#endif  // WAVERIDER_GEOMETRY_H_
