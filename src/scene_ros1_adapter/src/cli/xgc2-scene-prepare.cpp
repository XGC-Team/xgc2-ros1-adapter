#include "xgc_scene/prepare.hpp"

#include "xgc_scene/json_io.hpp"

#include <cerrno>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <map>
#include <regex>
#include <sys/stat.h>
#include <system_error>

namespace {
using xgc_scene::Json;
using xgc_scene::SceneError;
namespace fs = std::filesystem;

std::string environment(const char* name) {
  const auto value = std::getenv(name);
  return value ? value : "";
}

std::string compact(const Json& value) {
  if (value.is_string()) return value.dump(-1, ' ', true);
  if (!value.is_structured()) return xgc_scene::dumpJson(value);
  std::string result = value.is_object() ? "{" : "[";
  bool first = true;
  for (auto it = value.begin(); it != value.end(); ++it) {
    if (!first) result += ',';
    first = false;
    if (value.is_object()) result += compact(Json(it.key())) + ':';
    result += compact(it.value());
  }
  return result + (value.is_object() ? "}" : "]");
}

}  // namespace

int main(int argc, char** argv) {
  std::map<std::string, std::string> options{{"--resource-root", environment("XGC_USER_FILES_DIR")},
                                           {"--workspace-root", environment("XGC_JOB_ARTIFACT_ROOT")},
                                           {"--asset-directory", ""},
                                           {"--socket-path", ""}, {"--target-id", ""},
                                           {"--configuration-revision", ""}};
  bool prepare_world = false, validate = false, asset_directory_given = false;
  for (int i = 1; i < argc; ++i) {
    std::string argument(argv[i]);
    if (argument == "--help" || argument == "-h") {
      std::cout << "Usage: xgc2-scene-prepare --context-json JSON [--resource-root PATH] [--workspace-root PATH]\n"
                   "       [--prepare-world --asset-directory PATH --socket-path PATH --target-id ID --configuration-revision INTEGER]\n"
                   "       xgc2-scene-prepare --validate --asset-directory PATH\n"
                   "The context's expectedRevisionId, when present, must equal the asset manifest's revisionId.\n"
                   "--validate parses and dry-compiles PATH/scene-document.yaml; it writes nothing and exits 1 with the reason.\n";
      return 0;
    }
    if (argument == "--prepare-world") { prepare_world = true; continue; }
    if (argument == "--validate") { validate = true; continue; }
    const auto equals = argument.find('=');
    const auto name = argument.substr(0, equals);
    if ((name != "--context-json" && !options.count(name)) || (equals == std::string::npos && i + 1 == argc)) {
      std::cerr << "scene_prepare: unrecognized or incomplete argument " << name << '\n';
      return 2;
    }
    if (name == "--asset-directory") asset_directory_given = true;
    options[name] = equals == std::string::npos ? argv[++i] : argument.substr(equals + 1);
  }
  if (validate) {
    if (prepare_world || options.count("--context-json") || !asset_directory_given || options.at("--asset-directory").empty()) {
      std::cerr << "scene_prepare: --validate takes only --asset-directory\n";
      return 2;
    }
    try {
      std::cout << compact(xgc_scene::validate(options.at("--asset-directory"))) << '\n';
      return 0;
    } catch (const std::exception& error) {
      std::cerr << "scene_prepare: " << error.what() << '\n';
      return 1;
    }
  }
  if (!options.count("--context-json")) {
    std::cerr << "scene_prepare: the following arguments are required: --context-json\n";
    return 2;
  }
  Json revision;
  if (!options.at("--configuration-revision").empty()) {
    const auto text = options.at("--configuration-revision");
    static const std::regex integer("[+-]?[0-9]+");
    if (!std::regex_match(text, integer)) {
      std::cerr << "scene_prepare: --configuration-revision requires an integer\n";
      return 2;
    }
    try { revision = std::stoll(text); }
    catch (const std::out_of_range&) { revision = 0; }
  }
  try {
    const auto& raw = options.at("--context-json");
    if (raw.size() > 65536) throw SceneError("Scene context exceeds 64 KiB");
    // Python rejects literal NaN/Infinity, while a finite JSON number which
    // overflows is rejected by its eventual validator or encoder.
    bool quoted = false;
    for (std::size_t i = 0; i < raw.size(); ++i) {
      if (quoted && raw[i] == '\\') { ++i; continue; }
      if (raw[i] == '"') quoted = !quoted;
      if (!quoted && (raw.compare(i, 3, "NaN") == 0 || raw.compare(i, 8, "Infinity") == 0))
        throw SceneError("Nonfinite JSON");
    }
    const auto context = xgc_scene::parseJson(raw);
    Json value;
    if (!prepare_world) {
      if (asset_directory_given) throw SceneError("--asset-directory requires --prepare-world");
      value = xgc_scene::inputs(context, options.at("--resource-root"), options.at("--workspace-root"));
    } else {
      if (!asset_directory_given || options.at("--asset-directory").empty())
        throw SceneError("--prepare-world requires --asset-directory");
      const auto& scene = context.at("scene");
      if (scene.value("simulator", Json()) != "gazebo")
        throw SceneError("--prepare-world requires the gazebo simulator");
      auto authored = scene.value("parameters", Json::object());
      if (!authored.is_object()) throw SceneError("Scene parameters must be an object");
      authored.erase("lidar");
      bool locked = false;
      if (authored.contains("frozen")) {
        if (!authored.at("frozen").is_boolean()) throw SceneError("Scene parameter frozen must be a boolean");
        locked = authored.at("frozen").get<bool>();
        authored.erase("frozen");
      }
      const auto run = context.value("openingRunId", Json(""));
      static const std::regex identity("[A-Za-z0-9_-]{1,128}");
      if (!run.is_string() || !std::regex_match(run.get_ref<const std::string&>(), identity))
        throw SceneError("World materialization requires the actual Session opening Run identity");
      auto grant = fs::path(options.at("--workspace-root"));
      while (grant != grant.root_path() && grant.filename().empty()) grant = grant.parent_path();
      if (!grant.is_absolute() || !fs::is_directory(grant) || fs::is_symlink(grant))
        throw SceneError("An existing provider workspace grant is required");
      grant = fs::canonical(grant);
      const auto output = grant / "native-worlds" / run.get<std::string>();
      auto current = fs::canonical(grant);
      const auto relative = output.lexically_relative(current);
      if (relative.empty() || *relative.begin() == "..") throw SceneError("Resource path is outside the granted root");
      for (const auto& part : relative) {
        if (part == ".") continue;
        current /= part;
        if (fs::is_symlink(current)) throw SceneError("Provider workspace cannot contain symlinks");
        if (::mkdir(current.c_str(), 0750) != 0 && (errno != EEXIST || !fs::is_directory(current)))
          throw std::system_error(errno, std::generic_category(), current.string());
      }
      Json request{{"schema", xgc_scene::kPrepareSchema}, {"configuration_revision", revision},
                   {"resource_root", options.at("--resource-root")}, {"asset_directory", options.at("--asset-directory")},
                   {"socket_path", options.at("--socket-path")}, {"target_id", options.at("--target-id")},
                   {"output_grant", {{"directory", output.string()}, {"max_bytes", 16 * 1024 * 1024}}},
                   {"parameters", authored}, {"editable", !locked}, {"run_id", run}};
      // The revision the Session froze at open (sceneAsset.revisionId).
      const auto expected = context.value("expectedRevisionId", Json());
      if (!expected.is_null()) request["expected_revision_id"] = expected;
      value = xgc_scene::prepare(request);
      const auto manifest = xgc_scene::parseJson(xgc_scene::read(fs::path(value.at("assetDirectory").get<std::string>()) / "manifest.json", 65536));
      const auto document = manifest.value("sceneDocument", Json());
      value["sceneDocumentFile"] = document.is_object() ? document.value("file", "") : "";
      value["simulator"] = "gazebo";
      value["worldAttachment"] = !manifest.value("gazeboWorld", Json()).empty();
      value["generationParametersJson"] = compact(authored.value("generation", Json::object()));
      // Published consumer contract (see core inputs()): workflows bind facts.pointCloud.* of scene-inputs.
      if (manifest.contains("pointCloud")) value["pointCloud"] = manifest.at("pointCloud");
    }
    std::cout << compact(value) << '\n';
  } catch (const std::exception& error) {
    std::cerr << "scene_prepare: " << error.what() << '\n';
    return 1;
  }
  return 0;
}
