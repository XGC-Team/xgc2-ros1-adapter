#include "xgc_scene/yaml_io.hpp"

#include "xgc_scene/document.hpp"

#include <cerrno>
#include <fstream>
#include <system_error>

namespace xgc_scene {
namespace {

std::string read(const std::filesystem::path& path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) throw std::system_error(errno, std::generic_category(), path.string());
  std::string data(kMaxDocumentBytes + 1, '\0');
  stream.read(data.data(), data.size());
  if (stream.bad()) throw std::system_error(errno, std::generic_category(), path.string());
  data.resize(static_cast<std::size_t>(stream.gcount()));
  if (data.size() > kMaxDocumentBytes) throw SceneError("Scene file exceeds the supported size");
  return data;
}

}  // namespace

std::pair<Json, std::string> load(const std::filesystem::path& path) {
  const auto bytes = read(path);
  return {document(parseYaml(bytes)), digest(bytes)};
}
std::string fileDigest(const std::filesystem::path& path) { return digest(read(path)); }

}  // namespace xgc_scene
