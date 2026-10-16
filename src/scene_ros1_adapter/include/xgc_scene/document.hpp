#pragma once

#include "xgc_scene/common.hpp"

#include <string>
#include <vector>

namespace xgc_scene {

inline constexpr char kSchema[] = "xgc2.scene.v1";
inline constexpr char kCompiledSchema[] = "xgc2.scene.compiled.v1";

const Json& fields(const Json& value, const std::vector<std::string>& allowed,
                   const std::vector<std::string>& required = {});
double number(const Json& value, const std::string& label, double minimum = -1e6, double maximum = 1e6);
Json vector(const Json& value, const std::string& label, std::size_t count = 3);
std::string identifier(const Json& value, const std::string& label = "ID");
Json pose(const Json& value);
Json convex(const Json& value);
Json geometry(const Json& value);
Json motion(const Json& value);
Json obstacle(const Json& value);
Json models(const Json& value);
Json ground(const Json& value);
const Json& checkObstacleSet(const Json& obstacles);
Json document(const Json& value);
// Cold compiled.v1 projection. resource_reads counts SDF/mesh file opens inside this call.
// The cache lives only for this call; callers keep the returned value for an epoch revision.
Json compile(const Json& value, const std::string& resource_root = {}, std::size_t* resource_reads = nullptr);

}  // namespace xgc_scene
