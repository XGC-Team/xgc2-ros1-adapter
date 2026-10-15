#include "xgc_scene/store.hpp"

#include "xgc_scene/document.hpp"
#include "xgc_scene/json_io.hpp"
#include "xgc_scene/yaml_io.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <sys/stat.h>
#include <system_error>
#include <thread>
#include <unistd.h>

namespace xgc_scene {
namespace fs = std::filesystem;
namespace {
thread_local std::optional<std::string> replace_failure;
}
namespace detail {

// This executable supplies the static library's rename seam, just as the
// Python replay patches os.replace. Other tests use the real library seam.
void replaceStoreFile(const fs::path& temporary, const fs::path& target) {
  if (replace_failure) throw SceneError(*replace_failure);
  if (::rename(temporary.c_str(), target.c_str()) != 0)
    throw std::system_error(errno, std::generic_category());
}

}  // namespace detail

namespace {

std::string read(const fs::path& path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) throw std::runtime_error("Cannot read " + path.string());
  return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

void write(const fs::path& path, const std::string& bytes) {
  fs::create_directories(path.parent_path());
  std::ofstream stream(path, std::ios::binary);
  stream << bytes;
  if (!stream) throw std::runtime_error("Cannot write " + path.string());
}

class TemporaryDirectory {
 public:
  TemporaryDirectory() {
    char pattern[] = "/tmp/xgc-scene-store-XXXXXX";
    const auto directory = ::mkdtemp(pattern);
    if (!directory) throw std::runtime_error("Cannot create store test directory");
    path = directory;
  }
  ~TemporaryDirectory() { std::error_code error; fs::remove_all(path, error); }
  fs::path path;
};

void same(const Json& actual, const Json& expected) {
  if (expected.is_number()) {
    ASSERT_TRUE(actual.is_number());
    EXPECT_EQ(actual.is_number_float(), expected.is_number_float());
    EXPECT_EQ(actual.get<double>(), expected.get<double>());
    if (expected.is_number_float() && expected.get<double>() == 0) {
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

void sameError(const std::string& actual, const std::string& expected) {
  const std::string prefix = "Invalid scene YAML: ";
  if (expected.compare(0, prefix.size(), prefix) == 0) EXPECT_EQ(actual.substr(0, prefix.size()), prefix);
  else EXPECT_EQ(actual, expected);
}

class Replay {
 public:
  explicit Replay(const Json& scenario) : scenario_(scenario) { tokens_["$work"] = directory_.path.string(); }

  void run() {
    const auto& setup = scenario_.at("setup");
    for (auto it = setup.at("files").begin(); it != setup.at("files").end(); ++it) watched_.insert(it.key());
    for (const auto& directory : setup.at("directories")) fs::create_directories(bind(directory).get<std::string>());
    for (auto it = setup.at("files").begin(); it != setup.at("files").end(); ++it) materialize(bind(Json(it.key())), bind(it.value()));
    for (auto it = setup.at("stores").begin(); it != setup.at("stores").end(); ++it) {
      SCOPED_TRACE("initial " + it.key());
      auto options = it.value();
      options["initial"] = options.at("document");
      if (options.contains("initial_effects")) collectPaths(options.at("initial_effects"));
      open(it.key(), bind(options));
      if (options.contains("initial_response")) same(normalize(stores_.at(it.key())->envelope()), options.at("initial_response"));
      if (options.contains("initial_effects")) same(normalize(effects()), options.at("initial_effects"));
    }
    std::size_t index = 0;
    for (const auto& step : scenario_.at("steps")) {
      SCOPED_TRACE("step " + std::to_string(index++) + " " + step.at("operation").get<std::string>());
      const auto request = bind(step.at("request"));
      const auto label = step.value("store", std::string("main"));
      const auto operation = step.at("operation").get<std::string>();
      if (step.contains("effects")) collectPaths(step.at("effects"));
      if (step.contains("fault")) replace_failure = step.at("fault").at("message").get<std::string>();
      struct Restore { ~Restore() { replace_failure.reset(); } } restore;
      compareCall(step.at("response"), [&]() -> Json {
        if (operation == "store.command") return stores_.at(label)->command(request);
        if (operation == "store.envelope") return stores_.at(label)->envelope();
        if (operation == "store.states") return stores_.at(label)->states();
        if (operation == "store.restart") { open(label, request); return stores_.at(label)->envelope(); }
        if (operation == "clock.advance") { now_ += request.at("seconds").get<double>(); return now_; }
        if (operation == "simulator.apply_outcome") { apply_outcome_ = request; return nullptr; }
        if (operation == "file.mutate") { materialize(request.at("path"), request); return nullptr; }
        if (operation == "document.load") {
          const auto loaded = load(request.at("path").get<std::string>());
          return Json::array({loaded.first, loaded.second});
        }
        if (operation == "document.dump_yaml") {
          // Corpus object keys are sorted. Restore the writer's canonical
          // document order through the same validator as the recorder.
          const auto value = document(request);
          return Json{{"incremental", yaml_.dump(value)}, {"full", dumpYaml(value)}};
        }
        throw std::runtime_error("Unreplayed store operation " + operation);
      });
      replace_failure.reset();
      if (step.contains("effects")) same(normalize(effects()), step.at("effects"));
    }
  }

 private:
  const Json& scenario_;
  TemporaryDirectory directory_;
  std::map<std::string, std::string> tokens_;
  std::set<std::string> watched_;
  std::map<std::string, std::unique_ptr<SceneStore>> stores_;
  double now_ = 100;
  std::size_t epochs_ = 0;
  Json apply_outcome_{{"outcome", "ok"}};
  Json apply_calls_ = Json::array();
  Json motion_calls_ = Json::array();
  DocumentYaml yaml_;

  Json transform(const Json& value, bool reverse) const {
    std::vector<std::pair<std::string, std::string>> pairs;
    for (const auto& pair : tokens_) pairs.emplace_back(reverse ? pair.second : pair.first, reverse ? pair.first : pair.second);
    std::sort(pairs.begin(), pairs.end(), [](const auto& a, const auto& b) { return a.first.size() > b.first.size(); });
    const auto replace = [&](std::string text) {
      for (const auto& pair : pairs) {
        std::size_t at = 0;
        while ((at = text.find(pair.first, at)) != std::string::npos) {
          text.replace(at, pair.first.size(), pair.second);
          at += pair.second.size();
        }
      }
      return text;
    };
    std::function<Json(const Json&)> visit = [&](const Json& input) -> Json {
      if (input.is_string()) return replace(input.get<std::string>());
      if (input.is_object()) {
        Json output = Json::object();
        for (auto it = input.begin(); it != input.end(); ++it) output[replace(it.key())] = visit(it.value());
        return output;
      }
      if (input.is_array()) {
        Json output = Json::array();
        for (const auto& item : input) output.push_back(visit(item));
        return output;
      }
      return input;
    };
    return visit(value);
  }
  Json bind(const Json& value) const { return transform(value, false); }
  Json normalize(const Json& value) const { return transform(value, true); }

  void collectPaths(const Json& value) {
    if (!value.is_structured()) return;
    if (value.is_object() && value.contains("files")) {
      for (auto it = value.at("files").begin(); it != value.at("files").end(); ++it) watched_.insert(it.key());
    }
    for (const auto& item : value) collectPaths(item);
  }

  void materialize(const Json& path_value, const Json& observation) {
    const fs::path path = path_value.get<std::string>();
    fs::create_directories(path.parent_path());
    if (observation.contains("symlink")) {
      fs::remove(path);
      fs::create_symlink(observation.at("symlink").get<std::string>(), path);
    } else if (observation.contains("contents") && observation.at("contents").is_string()) {
      write(path, observation.at("contents").get<std::string>());
      if (observation.contains("mode")) { ASSERT_EQ(::chmod(path.c_str(), observation.at("mode").get<unsigned>()), 0); }
    } else fs::remove(path);
  }

  Json files() const {
    std::set<fs::path> paths;
    for (const auto& watched : watched_) paths.insert(bind(Json(watched)).get<std::string>());
    for (const auto& entry : fs::recursive_directory_iterator(directory_.path))
      if (entry.is_regular_file() || entry.is_symlink()) paths.insert(entry.path());
    Json result = Json::object();
    for (const auto& path : paths) {
      if (fs::is_symlink(path)) result[path.string()] = Json{{"symlink", fs::read_symlink(path).string()}};
      else if (fs::is_regular_file(path)) {
        struct stat info{};
        if (::stat(path.c_str(), &info) != 0) throw std::runtime_error("Cannot stat replay file");
        result[path.string()] = Json{{"contents", read(path)}, {"mode", info.st_mode & 0777}};
      } else result[path.string()] = Json{{"contents", nullptr}};
    }
    return result;
  }

  Json effects() {
    Json result{{"files", files()}, {"apply", apply_calls_}, {"motion", motion_calls_}};
    apply_calls_ = Json::array();
    motion_calls_ = Json::array();
    return result;
  }

  void open(const std::string& label, const Json& options) {
    const auto path = [&](const std::string& key) -> std::optional<fs::path> {
      if (!options.contains(key) || options.at(key).is_null()) return {};
      return fs::path(options.at(key).get<std::string>());
    };
    Json initial = options.at("initial");
    const auto source = path("source");
    if (source) initial = load(*source).first;
    SceneStore::Apply apply;
    if (options.value("apply_hook", false)) apply = [&](const Json& candidate, const std::string& epoch, SceneStore::Revision revision) {
      apply_calls_.push_back(Json{{"request", Json{{"document", candidate}, {"epoch", epoch}, {"revision", revision}}},
                                  {"response", apply_outcome_}, {"files", files()}});
      if (apply_outcome_.at("outcome") != "ok") throw SceneError(apply_outcome_.at("message").get<std::string>());
    };
    SceneStore::Motion motion;
    if (options.value("motion_hook", false)) motion = [&](const std::string& operation, const std::string& epoch, SceneStore::Revision revision) {
      motion_calls_.push_back(Json{{"request", Json{{"operation", operation}, {"epoch", epoch}, {"revision", revision}}},
                                   {"response", Json{{"outcome", "ok"}}}});
    };
    const auto sha = options.contains("source_digest") ? std::optional<std::string>(options.at("source_digest").get<std::string>()) : std::nullopt;
    auto store = std::make_unique<SceneStore>(initial, source, path("save_root"), apply, [&] { return now_; }, sha,
                                            options.value("frozen", false), path("working_file"), motion);
    const auto token = epochs_ == 0 ? "$epoch" : "$epoch" + std::to_string(epochs_);
    ++epochs_;
    tokens_[token] = store->status().at("epoch").get<std::string>();
    stores_[label] = std::move(store);
  }

  void compareCall(const Json& response, const std::function<Json()>& call) {
    try {
      const auto actual = normalize(call());
      ASSERT_TRUE(response.contains("value")) << "Expected exception: " << response.dump();
      same(actual, response.at("value"));
    } catch (const std::exception& error) {
      ASSERT_TRUE(response.contains("error")) << "Unexpected exception: " << error.what();
      sameError(normalize(Json(error.what())).get<std::string>(), response.at("error").at("message").get<std::string>());
    }
  }
};

std::vector<fs::path> scenarios() {
  std::vector<fs::path> result;
  for (const auto& entry : fs::directory_iterator(fs::path(XGC_SCENE_GOLDEN_DIR) / "corpus/store"))
    if (entry.path().extension() == ".json") result.push_back(entry.path());
  std::sort(result.begin(), result.end());
  return result;
}

class StoreReplay : public ::testing::TestWithParam<fs::path> {};
TEST_P(StoreReplay, CommandsHooksAndFiles) {
  const auto scenario = parseJson(read(GetParam()));
  SCOPED_TRACE(scenario.at("scenario").get<std::string>());
  Replay(scenario).run();
}
INSTANTIATE_TEST_SUITE_P(Corpus, StoreReplay, ::testing::ValuesIn(scenarios()), [](const auto& info) {
  auto name = info.param.stem().string();
  std::replace(name.begin(), name.end(), '-', '_');
  return name;
});

Json box(const std::string& id = "box") {
  return Json{{"id", id}, {"pose", Json{{"position", Json::array({1, 2, 3})}}},
              {"parts", Json::array({Json{{"id", "body"}, {"geometry", Json{{"type", "box"}, {"size", Json::array({1, 2, 3})}}}}})}};
}

Json scene(const Json& obstacles = Json::array()) {
  return Json{{"schema", kSchema}, {"id", "test"}, {"frame", "world"}, {"obstacles", obstacles}};
}

Json command(SceneStore& store, const std::string& id, const Json& operation, Json values = Json::object()) {
  const auto status = store.status();
  Json request{{"requestId", id}, {"expectedEpoch", status.at("epoch")}, {"expectedRevision", status.at("revision")}, {"operation", operation}};
  for (auto it = values.begin(); it != values.end(); ++it) request[it.key()] = it.value();
  return store.command(request);
}

TEST(Store, CorpusInventory) { EXPECT_EQ(scenarios().size(), 23u); }

TEST(Store, OneOf512ObstaclesIsValidatedAndEmittedAndSnapshotsStayImmutable) {
  TemporaryDirectory directory;
  const auto source = directory.path / "scene.yaml";
  Json items = Json::array();
  for (std::size_t i = 0; i < 512; ++i) items.push_back(box("o" + std::to_string(i)));
  write(source, dumpYaml(document(scene(items))));
  SceneStore store(load(source).first, source);
  ASSERT_EQ(command(store, "warm", "save").at("success"), true);
  EnvelopeJson encoder;
  const auto before = store.envelope();
  EXPECT_EQ(encoder.dumps(before), dumpJson(before));
  const auto validated = store.obstacleValidations();
  const auto emitted_yaml = store.yamlObstacleEmissions();
  EXPECT_EQ(emitted_yaml, 512u);
  const auto emitted_json = encoder.obstacleEmissions();
  auto moved = before.at("document").at("obstacles")[5];
  moved["pose"]["position"] = Json::array({9.0, 9.0, 9.0});
  auto result = command(store, "move", "update", Json{{"obstacle", moved}});
  ASSERT_EQ(result.at("success"), true);
  EXPECT_EQ(store.obstacleValidations() - validated, 1u);
  EXPECT_EQ(store.yamlObstacleEmissions() - emitted_yaml, 1u);
  EXPECT_EQ(encoder.dumps(result), dumpJson(result));
  EXPECT_EQ(encoder.obstacleEmissions() - emitted_json, 1u);
  EXPECT_EQ(read(source), dumpYaml(result.at("document")));
  EXPECT_EQ(before.at("document"), document(scene(items)));
  auto receipt = store.requestResult("move");
  ASSERT_TRUE(receipt);
  result["document"]["obstacles"][0]["name"] = "client mutation";
  EXPECT_NE(result, *store.requestResult("move"));
  EXPECT_EQ(command(store, "undo", "undo").at("document"), before.at("document"));
  EXPECT_EQ(load(source).first, before.at("document"));
  EXPECT_EQ(store.requestResult("move"), receipt);
}

TEST(Store, EnvelopesUseExactPythonTextIncludingCacheInvalidation) {
  EnvelopeJson encoder;
  const auto corpus = parseJson(read(fs::path(XGC_SCENE_GOLDEN_DIR) / "corpus/document/envelope-json.json"));
  for (const auto& step : corpus.at("steps")) {
    const auto value = parseJson(step.at("request").at("json").get<std::string>());
    const auto& response = step.at("response");
    if (response.contains("error")) {
      try { encoder.dumps(value); FAIL() << "Expected nonfinite number rejection"; }
      catch (const SceneError& error) { sameError(error.what(), response.at("error").at("message").get<std::string>()); }
    } else {
      EXPECT_EQ(encoder.dumps(value), response.at("value").get<std::string>());
      EXPECT_EQ(envelopeJson(value), response.at("value").get<std::string>());
    }
  }
  Json value{{"document", Json{{"obstacles", Json::array({Json{{"id", "item"}, {"number", 0.0}}})}}}};
  for (const auto& number : Json::array({0.0, -0.0, 1, 1.0})) {
    value["document"]["obstacles"][0]["number"] = number;
    EXPECT_EQ(encoder.dumps(value), dumpJson(value));
  }
  value["document"]["obstacles"][0] = Json{{"number", 1.0}, {"id", "item"}};
  EXPECT_EQ(encoder.dumps(value), dumpJson(value));
  value["document"]["obstacles"][0]["number"] = std::numeric_limits<double>::infinity();
  EXPECT_THROW(encoder.dumps(value), SceneError);
}

TEST(Store, YamlCachesDistinguishNumericTypesOrderAndEmptyDocuments) {
  DocumentYaml writer;
  Json value{{"schema", kSchema}, {"id", "test"}, {"frame", "world"},
             {"obstacles", Json::array({Json{{"id", "item"}, {"number", 0.0}}})}};
  for (const auto& number : Json::array({0.0, -0.0, 1, 1.0})) {
    value["obstacles"][0]["number"] = number;
    EXPECT_EQ(writer.dump(value), dumpYaml(value));
  }
  value["obstacles"][0] = Json{{"number", 1.0}, {"id", "item"}};
  EXPECT_EQ(writer.dump(value), dumpYaml(value));
  Json reordered{{"obstacles", value.at("obstacles")}, {"schema", kSchema}, {"id", "test"}, {"frame", "world"}};
  EXPECT_EQ(writer.dump(reordered), dumpYaml(reordered));
  EXPECT_EQ(writer.dump(scene()), dumpYaml(scene()));
}

TEST(Store, ReceiptLruHas256EntriesAndRetainsFailedCommands) {
  SceneStore store(scene());
  const auto status = store.status();
  Json first{{"requestId", "r0"}, {"expectedEpoch", status.at("epoch")}, {"expectedRevision", 1}, {"operation", "clear"}};
  const auto original = store.command(first);
  for (int i = 1; i < 256; ++i) ASSERT_EQ(command(store, "r" + std::to_string(i), "clear").at("success"), true);
  EXPECT_EQ(store.command(first), original);
  EXPECT_EQ(command(store, "last", "undo").at("success"), false);
  EXPECT_TRUE(store.requestResult("r0"));
  EXPECT_FALSE(store.requestResult("r1"));
  const auto failed = store.requestResult("last");
  ASSERT_TRUE(failed);
  EXPECT_EQ(failed->at("error"), "Nothing to undo");
  for (int i = 2; i < 256; ++i) EXPECT_TRUE(store.requestResult("r" + std::to_string(i)));
  first["expectedRevision"] = 2;
  EXPECT_EQ(store.command(first).at("error"), "Request ID was already used for a different operation");
  EXPECT_EQ(store.requestResult("r0"), std::optional<Json>(original));
}

TEST(Store, UndoRedoBoundAndReloadReset) {
  TemporaryDirectory directory;
  const auto source = directory.path / "scene.yaml";
  write(source, dumpYaml(document(scene(Json::array({box()})))));
  SceneStore store(load(source).first, source);
  for (int i = 0; i < 130; ++i) {
    auto moved = box();
    moved["pose"]["position"] = Json::array({i + 10.0, 2.0, 3.0});
    ASSERT_EQ(command(store, "move" + std::to_string(i), "update", Json{{"obstacle", moved}}).at("success"), true);
  }
  for (int i = 0; i < 128; ++i) ASSERT_EQ(command(store, "undo" + std::to_string(i), "undo").at("success"), true);
  EXPECT_EQ(command(store, "empty", "undo").at("error"), "Nothing to undo");
  EXPECT_EQ(store.envelope()["document"]["obstacles"][0]["pose"]["position"][0], 11.0);
  for (int i = 0; i < 128; ++i) ASSERT_EQ(command(store, "redo" + std::to_string(i), "redo").at("success"), true);
  EXPECT_EQ(command(store, "emptyredo", "redo").at("error"), "Nothing to redo");
  EXPECT_EQ(command(store, "reload", "reload").at("success"), true);
  EXPECT_EQ(command(store, "undoreloaded", "undo").at("error"), "Nothing to undo");
  EXPECT_EQ(command(store, "redoreloaded", "redo").at("error"), "Nothing to redo");
  SceneStore reopened(load(source).first, source);
  EXPECT_EQ(reopened.envelope().at("document"), store.envelope().at("document"));
  // Python persists the result of undo/redo, but starts empty history on reopen.
  EXPECT_EQ(command(reopened, "fresh", "undo").at("error"), "Nothing to undo");
}

TEST(Store, IdentityFencesAndEveryUnconfiguredOrUnsupportedChannel) {
  SceneStore store(scene(Json::array({box()})));
  SceneStore other(scene());
  const auto epoch = store.status().at("epoch").get<std::string>();
  EXPECT_NE(epoch, other.status().at("epoch"));
  ASSERT_EQ(epoch.size(), 32u);
  EXPECT_EQ(epoch[12], '4');
  EXPECT_NE(std::string("89ab").find(epoch[16]), std::string::npos);
  EXPECT_EQ(store.command(Json::array()).at("error"), "Expected an object");
  EXPECT_EQ(store.command(Json::object()).at("error"), "Invalid fields: unknown=[], missing=['operation']");
  EXPECT_EQ(store.command(Json{{"operation", "get"}, {"requestId", false}}).at("success"), true);
  for (const auto& id : Json::array({nullptr, false, 1, "", "space id", std::string(129, 'a')})) {
    auto request = Json{{"operation", "clear"}, {"requestId", id}};
    EXPECT_EQ(store.command(request).at("error"), "Request ID must follow the XRPC identity contract");
  }
  for (const auto& revision : Json::array({nullptr, false, true, 1.0, "1", -1, 2})) {
    auto request = Json{{"operation", "clear"}, {"requestId", "fence"}, {"expectedEpoch", epoch}, {"expectedRevision", revision}};
    EXPECT_EQ(store.command(request).at("error"), "Scene changed; refresh before editing");
  }
  EXPECT_FALSE(store.requestResult("fence"));
  EXPECT_EQ(command(store, "save", "save").at("error"), "This scene has no writable project directory configured");
  EXPECT_EQ(command(store, "reload", "reload").at("error"), "This scene has no source YAML configured");
  EXPECT_EQ(command(store, "unknown", "unknown").at("error"), "Unsupported scene operation 'unknown'");
  EXPECT_EQ(command(store, "null", nullptr).at("error"), "Unsupported scene operation None");
  EXPECT_EQ(command(store, "delete", "delete", Json{{"id", "absent"}}).at("error"), "Obstacle no longer exists");
  EXPECT_EQ(command(store, "update", "update", Json{{"obstacle", box("absent")}}).at("error"), "Obstacle no longer exists");
  EXPECT_EQ(command(store, "add", "add", Json{{"obstacle", box()}}).at("error"), "Obstacle ID already exists");
}

TEST(Store, SaveTargetGuardsAndNewWorkingFileMode) {
  TemporaryDirectory directory;
  const auto source = directory.path / "scene.yaml";
  write(source, dumpYaml(document(scene())));
  SceneStore outside(scene(), source, directory.path / "project");
  EXPECT_EQ(command(outside, "outside", "save").at("error"), "Save target is outside the configured project directory");
  SceneStore suffix(scene(), directory.path / "scene.txt");
  EXPECT_EQ(command(suffix, "suffix", "save").at("error"), "Save target must be a YAML file");
  SceneStore missing(scene(), directory.path / "missing" / "scene.yaml");
  EXPECT_EQ(command(missing, "missing", "save").at("error"), "Save directory does not exist");
  SceneStore no_source(scene(), {}, directory.path);
  EXPECT_EQ(command(no_source, "none", "save").at("error"), "This scene has no source YAML configured");
  const auto working = directory.path / "new" / "scene.YML";
  SceneStore store(scene(), source, directory.path, {}, {}, {}, false, working);
  EXPECT_EQ(command(store, "copy", "save").at("success"), true);
  struct stat info{};
  ASSERT_EQ(::stat(working.c_str(), &info), 0);
  EXPECT_EQ(info.st_mode & 0777, 0600u);
}

TEST(Store, HooksMayReadAndNestedCommandsAreRejectedAndMotionFailureDoesNotChangeTime) {
  SceneStore* live = nullptr;
  Json nested;
  std::vector<SceneStore::Revision> applied;
  double now = 100;
  SceneStore store(scene(Json::array({box()})), {}, {}, [&](const Json&, const std::string&, SceneStore::Revision revision) {
    applied.push_back(revision);
    if (live) {
      EXPECT_EQ(live->status()["configuration"]["applied"]["known"], false);
      EXPECT_EQ(live->envelope().at("revision"), 1);
      EXPECT_EQ(live->states().size(), 1u);
      nested = live->command(Json{{"operation", "get"}});
    }
  }, [&] { return now; }, {}, false, {}, [](const std::string&, const std::string&, SceneStore::Revision) {
    throw SceneError("motion reply lost");
  });
  live = &store;
  EXPECT_EQ(command(store, "resync", "resync").at("success"), true);
  EXPECT_EQ(applied, (std::vector<SceneStore::Revision>{1, 2}));
  EXPECT_EQ(nested.at("error"), "Scene commands cannot be issued from store hooks");
  EXPECT_FALSE(store.requestResult("nested"));
  EXPECT_EQ(command(store, "play", "play").at("error"), "motion reply lost");
  now += 5;
  EXPECT_EQ(store.envelope().at("sceneTime"), 0.0);
  EXPECT_EQ(store.status().at("playing"), false);
}

TEST(Store, PlaybackClampsBackwardsClockAndPreservesDefinition) {
  double now = 100;
  auto moving = box();
  moving["motion"] = Json{{"type", "constant_twist"}, {"linear", Json::array({2, 0, 0})}, {"angular", Json::array({0, 0, 0})}};
  SceneStore store(scene(Json::array({moving})), {}, {}, {}, [&] { return now; });
  const auto initial = store.envelope().at("document");
  EXPECT_EQ(command(store, "play", "play").at("success"), true);
  now = 99;
  EXPECT_EQ(store.sceneTime(), 0.0);
  now = 103;
  EXPECT_EQ(store.sceneTime(), 3.0);
  EXPECT_EQ(command(store, "pause", "pause").at("sceneTime"), 3.0);
  now = 120;
  EXPECT_EQ(store.sceneTime(), 3.0);
  EXPECT_EQ(command(store, "playagain", "play").at("sceneTime"), 3.0);
  now = 122;
  EXPECT_EQ(store.states()[0]["pose"]["position"][0], 11.0);
  EXPECT_EQ(command(store, "reset", "reset").at("sceneTime"), 0.0);
  EXPECT_EQ(store.envelope().at("document"), initial);
  EXPECT_EQ(store.status().at("dirty"), false);
}

TEST(Store, TwoThreadsIssueCommandsAndReadSnapshotsAndSharedEncoder) {
  SceneStore store(scene(Json::array({box()})));
  EnvelopeJson encoder;
  std::atomic<unsigned> accepted{0};
  std::atomic<bool> failed{false};
  const auto work = [&](int thread) {
    for (int i = 0; i < 150; ++i) {
      const auto id = "thread" + std::to_string(thread) + ":" + std::to_string(i);
      auto moved = box();
      moved["pose"]["position"] = Json::array({10.0 + thread * 150 + i, 2.0, 3.0});
      const auto result = command(store, id, "update", Json{{"obstacle", moved}});
      if (result.at("success") == true) ++accepted;
      else if (result.at("error") != "Scene changed; refresh before editing") failed = true;
      const auto envelope = store.envelope();
      if (envelope["configuration"]["desired"]["revision"] != envelope.at("revision") ||
          encoder.dumps(envelope) != dumpJson(envelope) || envelopeJson(envelope) != dumpJson(envelope) ||
          store.states().size() != 1 || store.command(Json{{"operation", "get"}}).at("success") != true) failed = true;
      const auto receipt = store.requestResult(id);
      if (result.at("success") == true && (!receipt || *receipt != result)) failed = true;
    }
  };
  std::thread first(work, 0), second(work, 1);
  first.join();
  second.join();
  EXPECT_FALSE(failed.load());
  EXPECT_GT(accepted.load(), 0u);
  EXPECT_EQ(store.status().at("revision"), accepted.load() + 1);
}

}  // namespace
}  // namespace xgc_scene
