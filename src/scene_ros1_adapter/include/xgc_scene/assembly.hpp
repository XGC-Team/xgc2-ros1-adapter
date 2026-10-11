#pragma once

#include "xgc_scene/common.hpp"

#include <string>

namespace xgc_scene {

Json assemble(const Json& request, const Json& documents, const Json& extrinsic);
// Decode the finite CLI input, enforcing its size and JSON error contract.
Json assembleJson(const std::string& bytes);

}  // namespace xgc_scene
