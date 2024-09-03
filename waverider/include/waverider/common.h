#ifndef WAVERIDER_COMMON_H_
#define WAVERIDER_COMMON_H_

#include <wavemap/core/common.h>
#include <wavemap/core/indexing/ndtree_index.h>

namespace waverider {
using FloatingPoint = wavemap::FloatingPoint;
using Point3D = wavemap::Point3D;
using Vector3D = wavemap::Vector3D;
using OctreeIndex = wavemap::OctreeIndex;

struct Plane3D {
  Vector3D normal;
  FloatingPoint offset;

  bool isBelow(const Point3D& point) const {
    return offset < normal.dot(point);
  }
};
}  // namespace waverider

#endif  // WAVERIDER_COMMON_H_
