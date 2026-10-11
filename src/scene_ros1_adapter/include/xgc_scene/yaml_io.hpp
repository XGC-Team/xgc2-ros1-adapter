#pragma once

#include "xgc_scene/common.hpp"

#include <filesystem>
#include <string>
#include <utility>

namespace xgc_scene {

Json parseYaml(const std::string& bytes);
std::string dumpYaml(const Json& value);
std::string digest(const std::string& bytes);
std::pair<Json, std::string> load(const std::filesystem::path& path);
std::string fileDigest(const std::filesystem::path& path);

}  // namespace xgc_scene
