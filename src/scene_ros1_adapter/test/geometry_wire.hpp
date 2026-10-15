#pragma once
#include <xgc2_math/geometry/geometry_json.hpp>
#include <xgc2_geometry_msgs/SceneGeometry.h>
namespace geometry_test {
using xgc2_math::geometry::Geometry;
using xgc2_math::geometry::GeometryType;
inline xgc2_geometry_msgs::SceneGeometry wire(const Geometry& geometry) {
  const auto payload = xgc2_math::geometry::encodeGeometry(geometry);
  xgc2_geometry_msgs::SceneGeometry result;
  result.type = payload.type; result.parameters = payload.parameters;
  return result;
}
inline Geometry read(const xgc2_geometry_msgs::SceneGeometry& geometry) {
  return xgc2_math::geometry::decodeGeometry({geometry.type, geometry.parameters});
}
}
