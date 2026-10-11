#pragma once

#include "xgc_scene/common.hpp"

#include <filesystem>
#include <string>
#include <utility>
#include <vector>

namespace xgc_scene {

std::string run_generator(const std::vector<std::string>& argv, std::size_t output_limit);
std::pair<std::pair<Json, std::string>, std::string> resolve(
    const std::filesystem::path& source, const std::filesystem::path& working_file = {},
    const Json& overrides = nullptr, std::size_t max_bytes = kMaxDocumentBytes);

}  // namespace xgc_scene
