#pragma once

#include "xgc_scene/common.hpp"

#include <filesystem>
#include <string>
#include <utility>

namespace xgc_scene {

inline constexpr char kPrepareSchema[] = "xgc2.simulation.prepare.v1";
inline constexpr char kDefaultWorld[] =
    "/opt/ros/noetic/share/gazebo_sim_worlds/worlds/scene_editable/scene_editable.world";

std::string read(const std::filesystem::path& path, std::size_t limit);
std::filesystem::path checkedPath(const Json& value, const std::filesystem::path& root,
                                  bool directory = false);
std::string sha256(const std::filesystem::path& path);
std::string attachment(const std::filesystem::path& directory, const Json& value,
                       bool required = false);
Json parameters(const Json& value, bool template_world);
void atomic(const std::filesystem::path& path, const std::string& data);
Json prepare(const Json& request);
Json inputs(const Json& context, const std::filesystem::path& resource_root,
            const std::filesystem::path& workspace_root);
// Finite write-path check of the sealed document in <directory>/scene-document.yaml:
// the same document() validation and a dry compile (resources are read from
// <directory>) that every Session runs in inputs(). Nothing is written and no
// generator runs. Throws SceneError (or a parser/IO exception) with the reason.
Json validate(const std::filesystem::path& directory);

}  // namespace xgc_scene

// The sibling generation package owns this function and its implementation.
// Python: resolve(source, working_file='', overrides=None, *, max_bytes=MAX_DOCUMENT_BYTES).
#if __has_include("xgc_scene/generation.hpp")
#include "xgc_scene/generation.hpp"
#else
namespace xgc_scene {
std::pair<std::pair<Json, std::string>, std::string> resolve(
    const std::filesystem::path& source, const std::filesystem::path& working_file = {},
    const Json& overrides = nullptr, std::size_t max_bytes = kMaxDocumentBytes);
}  // namespace xgc_scene
#endif
