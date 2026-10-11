#include <cerrno>
#include <cstdio>
#include <filesystem>
#include <system_error>

namespace xgc_scene::detail {

// Kept at the IO seam so replay tests can inject the Python rename failure
// without adding a transport or filesystem option to SceneStore.
void replaceStoreFile(const std::filesystem::path& temporary, const std::filesystem::path& target) {
  if (::rename(temporary.c_str(), target.c_str()) != 0)
    throw std::system_error(errno, std::generic_category());
}

}  // namespace xgc_scene::detail
