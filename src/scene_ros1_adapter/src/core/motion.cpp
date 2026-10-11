#include "xgc_scene/motion.hpp"

#include "placement_json.hpp"

namespace xgc_scene {

Json multiply(const Json& a, const Json& b) {
  return detail::fromQuaternion(xgc2_math::scene::multiply(detail::toQuaternion(a), detail::toQuaternion(b)));
}
Json rotate(const Json& quaternion, const Json& value) {
  return detail::fromVector(xgc2_math::scene::rotate(detail::toQuaternion(quaternion), detail::toVector(value)));
}
Json compose(const Json& a, const Json& b) {
  return detail::fromPose(xgc2_math::scene::composePlacement(detail::toPose(a), detail::toPose(b)));
}
Json state(const Json& obstacle, double elapsed, bool playing) {
  const auto result = xgc2_math::scene::evaluateMotion(detail::toPose(obstacle.at("pose")),
                                                     detail::toMotion(obstacle.at("motion")), elapsed, playing);
  return Json{{"id", obstacle.at("id")}, {"pose", detail::fromPose(result.pose)},
              {"linear", detail::fromVector(result.linear)}, {"angular", detail::fromVector(result.angular)}};
}

}  // namespace xgc_scene
