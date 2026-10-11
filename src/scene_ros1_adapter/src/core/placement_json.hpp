#pragma once

#include "xgc_scene/common.hpp"

#include <xgc2_math/scene/placement.hpp>

namespace xgc_scene::detail {

inline Eigen::Vector3d toVector(const Json& value) {
  return {value.at(0).get<double>(), value.at(1).get<double>(), value.at(2).get<double>()};
}
inline Eigen::Quaterniond toQuaternion(const Json& value) {
  return {value.at(3).get<double>(), value.at(0).get<double>(), value.at(1).get<double>(), value.at(2).get<double>()};
}
inline xgc2_math::Pose3 toPose(const Json& value) {
  xgc2_math::Pose3 result;
  result.position = toVector(value.at("position"));
  result.orientation = toQuaternion(value.at("orientation"));
  return result;
}
inline Json fromVector(const Eigen::Vector3d& value) { return Json::array({value.x(), value.y(), value.z()}); }
inline Json fromQuaternion(const Eigen::Quaterniond& value) { return Json::array({value.x(), value.y(), value.z(), value.w()}); }
inline Json fromPose(const xgc2_math::Pose3& value) {
  return Json{{"position", fromVector(value.position)}, {"orientation", fromQuaternion(value.orientation)}};
}
inline xgc2_math::scene::Motion toMotion(const Json& value) {
  xgc2_math::scene::Motion result;
  const auto& type = value.at("type");
  if (type == "constant_twist") {
    result.type = xgc2_math::scene::Motion::kConstantTwist;
    result.linear = toVector(value.at("linear"));
    result.angular = toVector(value.at("angular"));
  } else if (type == "ping_pong") {
    result.type = xgc2_math::scene::Motion::kPingPong;
    result.point_a = toVector(value.at("point_a"));
    result.point_b = toVector(value.at("point_b"));
    result.speed = value.at("speed").get<double>();
  } else if (type == "circle") {
    result.type = xgc2_math::scene::Motion::kCircle;
    result.center = toVector(value.at("center"));
    result.radius = value.at("radius").get<double>();
    result.angular_speed = value.at("angular_speed").get<double>();
    result.phase = value.at("phase").get<double>();
  }
  return result;
}

}  // namespace xgc_scene::detail
