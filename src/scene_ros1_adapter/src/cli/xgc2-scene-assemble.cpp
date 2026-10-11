#include "xgc_scene/assembly.hpp"

#include <yaml-cpp/yaml.h>

#include <iostream>
#include <stdexcept>
#include <string>

namespace {

int fail(const char* message) {
  std::cerr << message << '\n';
  return 2;
}

}  // namespace

int main() {
  try {
    std::string raw(4 * xgc_scene::kMaxDocumentBytes + 1, '\0');
    std::cin.read(raw.data(), static_cast<std::streamsize>(raw.size()));
    raw.resize(static_cast<std::size_t>(std::cin.gcount()));
    const auto result = xgc_scene::assembleJson(raw);
    // The result has only string values; escaping matches json.dumps' default
    // ensure_ascii=True and its compact separators, with no terminal newline.
    std::cout << result.dump(-1, ' ', true);
    return 0;
  } catch (const xgc_scene::SceneError& error) { return fail(error.what()); }
  catch (const YAML::Exception& error) { return fail(error.what()); }
  catch (const xgc_scene::Json::exception& error) { return fail(error.what()); }
  catch (const std::invalid_argument& error) { return fail(error.what()); }
  catch (const std::out_of_range& error) { return fail(error.what()); }
}
