#pragma once

#include "xgc_scene/common.hpp"

namespace xgc_scene {

Json multiply(const Json& a, const Json& b);
Json rotate(const Json& quaternion, const Json& value);
Json compose(const Json& a, const Json& b);
// The obstacle must already be normalized by obstacle() or document().
Json state(const Json& obstacle, double elapsed, bool playing);

}  // namespace xgc_scene
