#include "xgc_scene/document.hpp"
#include "xgc_scene/json_io.hpp"
#include "xgc_scene/motion.hpp"
#include "xgc_scene/yaml_io.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <set>
#include <unistd.h>

namespace xgc_scene {
namespace {

namespace fs = std::filesystem;

std::vector<fs::path> scenarios() {
  std::vector<fs::path> result;
  for (const auto& category : fs::directory_iterator(fs::path(XGC_SCENE_GOLDEN_DIR) / "corpus")) {
    if (!category.is_directory()) continue;
    for (const auto& file : fs::directory_iterator(category.path()))
      if (file.path().extension() == ".json") result.push_back(file.path());
  }
  std::sort(result.begin(), result.end());
  return result;
}

std::string read(const fs::path& path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) throw std::runtime_error("Cannot read corpus file " + path.string());
  return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

void same(const Json& actual, const Json& expected) {
  if (expected.is_number()) {
    ASSERT_TRUE(actual.is_number());
    EXPECT_EQ(actual.is_number_float(), expected.is_number_float());
    EXPECT_EQ(actual.get<double>(), expected.get<double>());
    if (expected.get<double>() == 0 && expected.is_number_float()) {
      EXPECT_EQ(std::signbit(actual.get<double>()), std::signbit(expected.get<double>()));
    }
  } else if (expected.is_structured()) {
    ASSERT_EQ(actual.type(), expected.type());
    ASSERT_EQ(actual.size(), expected.size());
    if (expected.is_object()) {
      for (auto it = expected.begin(); it != expected.end(); ++it) {
        SCOPED_TRACE(it.key());
        ASSERT_TRUE(actual.contains(it.key()));
        same(actual.at(it.key()), it.value());
      }
    } else {
      for (std::size_t i = 0; i < expected.size(); ++i) { SCOPED_TRACE(i); same(actual[i], expected[i]); }
    }
  } else EXPECT_EQ(actual, expected);
}

std::string recordedError(const Json& response) {
  if (response.contains("error")) return response.at("error").at("message").get<std::string>();
  if (response.contains("value") && response.at("value").is_object()) {
    const auto& value = response.at("value");
    if (value.contains("error")) return value.at("error").get<std::string>();
    if (value.contains("stderr")) {
      auto message = value.at("stderr").get<std::string>();
      if (!message.empty() && message.back() == '\n') message.pop_back();
      return message;
    }
  }
  return {};
}

void sameError(const std::string& actual, const std::string& expected) {
  // libyaml and yaml-cpp supply different parser diagnostics. Only this
  // prefix is stable; all validation errors are compared in full.
  const std::string prefix = "Invalid scene YAML: ";
  if (expected.compare(0, prefix.size(), prefix) == 0) EXPECT_EQ(actual.substr(0, prefix.size()), prefix);
  else EXPECT_EQ(actual, expected);
}

void replayCall(const Json& response, const std::function<Json()>& call) {
  try {
    const auto actual = call();
    ASSERT_TRUE(response.contains("value")) << "Expected error " << recordedError(response);
    same(actual, response.at("value"));
  } catch (const SceneError& error) { sameError(error.what(), recordedError(response)); }
}

bool yamlPath(const std::string& path) {
  const auto extension = fs::path(path).extension();
  return extension == ".yaml" || extension == ".yml";
}

class Replay {
  struct StoreSample {
    Json document;
    double scene_time = 0;
    double clock = 100;
    bool playing = false;
  };

  const Json& scenario_;
  fs::path work_;
  std::map<std::string, std::string> files_;
  std::set<std::string> runtime_files_;
  std::set<std::string> canonical_inputs_;
  std::map<std::string, StoreSample> stores_;
  double now_ = 100;
  int input_documents_ = 0;
  int yaml_outputs_ = 0;
  int yaml_checks_ = 0;
  int motion_samples_ = 0;
  int direct_calls_ = 0;

  fs::path boundPath(std::string path) {
    const auto at = path.find("$work");
    if (at == std::string::npos) throw std::runtime_error("Unexpected corpus path " + path);
    path.replace(at, 5, work_.string());
    return path;
  }
  void materialize(const std::string& path, const std::string& text) {
    const auto target = boundPath(path);
    fs::create_directories(target.parent_path());
    std::ofstream stream(target, std::ios::binary);
    stream << text;
    if (!stream) throw std::runtime_error("Cannot materialize corpus file " + path);
  }

  Json inputDocument(const Json& value, const Json& response = Json::object()) {
    ++input_documents_;
    try { return document(value); }
    catch (const SceneError& error) { sameError(error.what(), recordedError(response)); return nullptr; }
  }
  bool inputYaml(const std::string& text, const Json& response = Json::object()) {
    // Source manifests and opaque media files are not scene documents.
    if (text.find("xgc2.scene.v1") == std::string::npos && text.find("xgc2.scene-source.v1") == std::string::npos) return false;
    try {
      const auto parsed = parseYaml(text);
      if (parsed.is_object() && parsed.value("schema", Json(nullptr)) == kSchema) inputDocument(parsed, response);
      // The recorder creates these inputs with PyYAML too. An external edit
      // adding a comment is retained input text, not a runtime serialization.
      return text.compare(0, 1, "#") != 0;
    } catch (const SceneError& error) { sameError(error.what(), recordedError(response)); return false; }
  }
  void outputYaml(const std::string& text) {
    ++yaml_outputs_; ++yaml_checks_;
    const auto parsed = parseYaml(text);
    EXPECT_EQ(dumpYaml(parsed), text);
    if (parsed.is_object() && parsed.value("schema", Json(nullptr)) == kSchema) { EXPECT_NO_THROW(document(parsed)); }
  }

  void documentsInResponse(const Json& value) {
    if (!value.is_structured()) return;
    if (value.is_object() && value.value("schema", Json(nullptr)) == kSchema) {
      EXPECT_NO_THROW(document(value));
      return;
    }
    for (const auto& item : value) documentsInResponse(item);
  }
  void observeFiles(const Json& effects) {
    if (effects.contains("apply")) for (const auto& hook : effects.at("apply")) observeFiles(hook);
    if (!effects.contains("files")) return;
    for (auto it = effects.at("files").begin(); it != effects.at("files").end(); ++it) {
      SCOPED_TRACE(it.key());
      const auto& observation = it.value();
      if (!observation.contains("contents") || !observation.at("contents").is_string()) continue;
      const auto text = observation.at("contents").get<std::string>();
      const auto previous = files_.find(it.key());
      if (previous == files_.end() || previous->second != text) runtime_files_.insert(it.key());
      // Observations also contain unchanged external inputs (including invalid
      // YAML and opaque media). Track provenance so every runtime YAML output
      // is checked without treating those inputs as emitted runtime bytes.
      if (yamlPath(it.key()) && runtime_files_.count(it.key())) outputYaml(text);
      else if (yamlPath(it.key()) && canonical_inputs_.count(it.key())) {
        ++yaml_checks_;
        EXPECT_EQ(dumpYaml(parseYaml(text)), text);
      }
      files_[it.key()] = text;
      materialize(it.key(), text);
    }
  }
  void capture(const std::string& label, const Json& envelope) {
    if (!envelope.is_object() || !envelope.contains("document")) return;
    auto& sample = stores_[label];
    sample.document = envelope.at("document");
    sample.scene_time = envelope.at("sceneTime").get<double>();
    sample.playing = envelope.at("playing").get<bool>();
    sample.clock = now_;
  }

  void commandInput(const Json& request, const Json& response, const std::string& label) {
    if (request.contains("document")) inputDocument(request.at("document"), response);
    if (!request.contains("obstacle")) return;
    Json normalized;
    try { normalized = obstacle(request.at("obstacle")); }
    catch (const SceneError& error) { sameError(error.what(), recordedError(response)); return; }
    // Feed the scene-wide guard from the captured baseline; no store command,
    // persistence, simulator or revision behavior is implemented by this test.
    Json candidates = stores_.at(label).document.at("obstacles");
    const auto type = request.at("operation");
    auto existing = std::find_if(candidates.begin(), candidates.end(), [&](const auto& item) {
      return item.at("id") == normalized.at("id");
    });
    if (type == "add" && existing == candidates.end()) candidates.push_back(normalized);
    else if (type == "update" && existing != candidates.end()) *existing = normalized;
    else return;
    try { checkObstacleSet(candidates); }
    catch (const SceneError& error) { sameError(error.what(), recordedError(response)); }
  }

 public:
  explicit Replay(const Json& scenario) : scenario_(scenario) {
    char directory[] = "/tmp/xgc-scene-replay-XXXXXX";
    const auto path = mkdtemp(directory);
    if (!path) throw std::runtime_error("Cannot create corpus replay directory");
    work_ = path;
  }
  ~Replay() { fs::remove_all(work_); }

  void run() {
    const auto& setup = scenario_.at("setup");
    for (auto it = setup.at("files").begin(); it != setup.at("files").end(); ++it) {
      if (!it.value().contains("contents")) continue;
      const auto text = it.value().at("contents").get<std::string>();
      files_[it.key()] = text;
      materialize(it.key(), text);
      Json response = Json::object();
      for (const auto& step : scenario_.at("steps")) {
        if (step.at("operation") == "document.load" && step.at("request").at("path") == it.key()) {
          response = step.at("response"); break;
        }
      }
      if (yamlPath(it.key()) && inputYaml(text, response)) canonical_inputs_.insert(it.key());
    }
    for (auto it = setup.at("stores").begin(); it != setup.at("stores").end(); ++it) {
      SCOPED_TRACE(it.key());
      const auto normalized = inputDocument(it.value().at("document"));
      stores_[it.key()].document = normalized;
      if (it.value().contains("initial_response")) {
        same(normalized, it.value().at("initial_response").at("document"));
        capture(it.key(), it.value().at("initial_response"));
      }
      if (it.value().contains("initial_effects")) observeFiles(it.value().at("initial_effects"));
    }
    for (std::size_t index = 0; index < scenario_.at("steps").size(); ++index) {
      SCOPED_TRACE(index);
      const auto& step = scenario_.at("steps")[index];
      const auto& request = step.at("request");
      const auto& response = step.at("response");
      const auto operation = step.at("operation").get<std::string>();
      SCOPED_TRACE(operation);
      const auto label = step.value("store", std::string("main"));
      if (operation == "document.validate") {
        ++direct_calls_; ++input_documents_;
        replayCall(response, [&] { return document(request); });
      } else if (operation == "document.geometry") {
        ++direct_calls_;
        replayCall(response, [&] { return geometry(parseJson(request.at("json").get<std::string>())); });
      } else if (operation == "document.parse_json") {
        ++direct_calls_;
        replayCall(response, [&] { return parseJson(request.at("json").get<std::string>()); });
      } else if (operation == "document.load") {
        ++direct_calls_;
        replayCall(response, [&] {
          const auto result = load(boundPath(request.at("path").get<std::string>()));
          return Json::array({result.first, result.second});
        });
      } else if (operation == "envelope.encode") {
        ++direct_calls_;
        replayCall(response, [&] { return Json(dumpJson(parseJson(request.at("json").get<std::string>()))); });
      } else if (operation == "document.dump_yaml") {
        ++direct_calls_;
        // The corpus sorts JSON object keys. Revalidation restores the
        // canonical dict order that the Python writer received.
        const auto text = dumpYaml(document(request));
        EXPECT_EQ(text, response.at("value").at("full"));
        EXPECT_EQ(text, response.at("value").at("incremental"));
        inputDocument(request);
        outputYaml(response.at("value").at("full").get<std::string>());
      } else if (operation == "motion.state") {
        ++direct_calls_; ++motion_samples_;
        replayCall(response, [&] { return state(request.at("obstacle"), request.at("elapsed").get<double>(), request.at("playing").get<bool>()); });
      } else if (operation == "store.command") commandInput(request, response, label);
      else if (operation == "store.restart") inputDocument(request.at("initial"), response);
      else if (operation == "store.states") {
        const auto& sample = stores_.at(label);
        const double elapsed = sample.scene_time + (sample.playing ? now_ - sample.clock : 0);
        Json states = Json::array();
        for (const auto& item : sample.document.at("obstacles")) {
          states.push_back(state(item, elapsed, sample.playing)); ++motion_samples_;
        }
        same(states, response.at("value"));
      } else if (operation == "clock.advance") now_ = response.at("value").get<double>();
      else if (operation == "file.mutate" && request.contains("contents")) {
        const auto path = request.at("path").get<std::string>(), text = request.at("contents").get<std::string>();
        files_[path] = text; runtime_files_.erase(path); materialize(path, text);
        canonical_inputs_.erase(path);
        if (yamlPath(path) && inputYaml(text)) canonical_inputs_.insert(path);
      } else if (operation == "assembly.cli") {
        // Extract its input/output documents; the assembly CLI itself belongs
        // to the following package, as do the other categories' operations.
        Json input;
        try { input = parseJson(request.at("stdin").get<std::string>()); }
        catch (const std::exception&) { input = nullptr; }
        if (input.is_object() && input.contains("documents") && input.at("documents").is_array())
          for (const auto& text : input.at("documents")) inputYaml(text.get<std::string>(), response);
        const auto output = response.at("value").at("stdout").get<std::string>();
        if (!output.empty()) outputYaml(parseJson(output).at("sceneDocument").get<std::string>());
      } else if (operation == "http.call" && request.at("body").is_object()) {
        const auto& body = request.at("body");
        if (body.contains("document")) inputDocument(body.at("document"), response);
        if (body.contains("obstacle")) { EXPECT_NO_THROW(obstacle(body.at("obstacle"))); }
      }
      documentsInResponse(response);
      if (response.contains("value")) capture(label, response.at("value"));
      if (step.contains("effects")) {
        documentsInResponse(step.at("effects"));
        observeFiles(step.at("effects"));
      }
    }
    ::testing::Test::RecordProperty("input_documents", input_documents_);
    ::testing::Test::RecordProperty("yaml_outputs", yaml_outputs_);
    ::testing::Test::RecordProperty("yaml_checks", yaml_checks_);
    ::testing::Test::RecordProperty("motion_samples", motion_samples_);
    ::testing::Test::RecordProperty("direct_calls", direct_calls_);
  }
};

TEST(GoldenInventory, AllSixtyScenariosArePresent) {
  const auto files = scenarios();
  ASSERT_EQ(files.size(), 60u);
  std::map<std::string, int> categories;
  for (const auto& file : files) ++categories[file.parent_path().filename().string()];
  EXPECT_EQ(categories["document"], 10);
  EXPECT_EQ(categories["motion"], 6);
  EXPECT_EQ(categories.size(), 7u);
}

class GoldenReplay : public ::testing::TestWithParam<fs::path> {};
TEST_P(GoldenReplay, DocumentMotionAndYaml) {
  const auto scenario = parseJson(read(GetParam()));
  SCOPED_TRACE(scenario.at("scenario").get<std::string>());
  Replay(scenario).run();
}
INSTANTIATE_TEST_SUITE_P(Corpus, GoldenReplay, ::testing::ValuesIn(scenarios()), [](const auto& info) {
  auto name = info.param.parent_path().filename().string() + "_" + info.param.stem().string();
  std::replace(name.begin(), name.end(), '-', '_');
  return name;
});

}  // namespace
}  // namespace xgc_scene
