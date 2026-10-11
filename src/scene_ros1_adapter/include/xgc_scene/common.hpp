#pragma once
// Shared vocabulary of the scene adapter's ROS-free authoring core.
#include <nlohmann/json.hpp>

#include <cstddef>
#include <stdexcept>

namespace xgc_scene {

// The document model: the same shapes as the dicts of the Python scene runtime, kept as JSON values.
using Json = nlohmann::json;

// Every validation and command failure of the authoring core; the text is the wire-visible error message.
class SceneError : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};

// Largest document, file or request body the core reads (bytes).
constexpr std::size_t kMaxDocumentBytes = 8 * 1024 * 1024;

}  // namespace xgc_scene
