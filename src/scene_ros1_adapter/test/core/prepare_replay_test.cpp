#include "xgc_scene/prepare.hpp"

#include "xgc_scene/json_io.hpp"
#include "xgc_scene/yaml_io.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <map>
#include <sys/stat.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

namespace xgc_scene {
namespace {
namespace fs = std::filesystem;

std::string bytes(const fs::path& path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) throw std::runtime_error("Cannot read " + path.string());
  return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

void write(const fs::path& path, const std::string& text, unsigned mode = 0644) {
  fs::create_directories(path.parent_path());
  std::ofstream stream(path, std::ios::binary);
  stream << text;
  stream.close();
  if (!stream || ::chmod(path.c_str(), mode) != 0) throw std::runtime_error("Cannot write " + path.string());
}

void replace(std::string& text, const std::string& from, const std::string& to) {
  for (std::size_t at = 0; (at = text.find(from, at)) != std::string::npos; at += to.size()) text.replace(at, from.size(), to);
}

void same(const Json& actual, const Json& expected) {
  ASSERT_EQ(actual, expected);
  if (expected.is_number()) EXPECT_EQ(actual.is_number_float(), expected.is_number_float());
  else if (expected.is_object()) for (auto it = expected.begin(); it != expected.end(); ++it) {
    SCOPED_TRACE(it.key()); same(actual.at(it.key()), it.value());
  } else if (expected.is_array()) for (std::size_t i = 0; i < expected.size(); ++i) {
    SCOPED_TRACE(i); same(actual.at(i), expected.at(i));
  }
}

struct Temporary {
  fs::path root;
  Temporary() {
    char name[] = "/tmp/xgc-prepare-XXXXXX";
    if (!::mkdtemp(name)) throw std::runtime_error("Cannot create preparation fixture");
    root = name;
  }
  ~Temporary() { std::error_code error; fs::remove_all(root, error); }
};

struct ProcessResult { int code; std::string out, err; };

ProcessResult cli(const fs::path& capture, const std::vector<std::string>& arguments,
                  const std::map<std::string, std::string>& environment = {}) {
  std::array<char, 4096> executable{};
  const auto count = ::readlink("/proc/self/exe", executable.data(), executable.size() - 1);
  if (count < 0) throw std::runtime_error("Cannot locate replay executable");
  const auto binary = fs::path(std::string(executable.data(), count)).parent_path() / "xgc2-scene-prepare";
  std::vector<std::string> argv{binary.string()};
  argv.insert(argv.end(), arguments.begin(), arguments.end());
  std::vector<char*> pointers;
  for (auto& argument : argv) pointers.push_back(argument.data());
  pointers.push_back(nullptr);
  const auto output = capture / "stdout", error = capture / "stderr";
  const int out_fd = ::open(output.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0600);
  const int err_fd = ::open(error.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0600);
  if (out_fd < 0 || err_fd < 0) {
    if (out_fd >= 0) ::close(out_fd);
    if (err_fd >= 0) ::close(err_fd);
    throw std::runtime_error("Cannot capture preparation CLI");
  }
  const auto pid = ::fork();
  if (pid == 0) {
    ::setpgid(0, 0);
    if (::dup2(out_fd, STDOUT_FILENO) < 0 || ::dup2(err_fd, STDERR_FILENO) < 0) ::_exit(126);
    ::close(out_fd); ::close(err_fd);
    for (const auto& value : environment) ::setenv(value.first.c_str(), value.second.c_str(), 1);
    ::execv(binary.c_str(), pointers.data());
    ::_exit(127);
  }
  ::close(out_fd); ::close(err_fd);
  if (pid < 0) throw std::runtime_error("Cannot start preparation CLI");
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(45);
  int status = 0;
  while (true) {
    const auto waited = ::waitpid(pid, &status, WNOHANG);
    if (waited == pid) break;
    if (waited < 0 && errno != EINTR) throw std::runtime_error("Cannot wait for preparation CLI");
    if (std::chrono::steady_clock::now() >= deadline) {
      ::kill(-pid, SIGKILL); ::kill(pid, SIGKILL);
      while (::waitpid(pid, &status, 0) < 0 && errno == EINTR) {}
      throw std::runtime_error("Preparation CLI exceeded the replay budget");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return {WIFEXITED(status) ? WEXITSTATUS(status) : 128 + WTERMSIG(status), bytes(output), bytes(error)};
}

std::vector<std::string> scenarios() {
  std::vector<std::string> result;
  for (const auto* origin : {"corpus", "authored"})
    for (const auto& file : fs::directory_iterator(fs::path(XGC_SCENE_GOLDEN_DIR) / origin / "prepare"))
      if (file.path().extension() == ".json") result.push_back(file.path().stem().string());
  std::sort(result.begin(), result.end());
  return result;
}

struct Fixture {
  Temporary temporary;
  fs::path work = temporary.root / "work", fixtures = temporary.root / "fixtures";
  Json scenario;
  std::map<std::string, std::string> bindings;
  explicit Fixture(const std::string& name, bool use_cli = false) {
    auto path = fs::path(XGC_SCENE_GOLDEN_DIR) / "corpus" / "prepare" / (name + ".json");
    const bool recorded = fs::exists(path);
    if (!recorded) path = fs::path(XGC_SCENE_GOLDEN_DIR) / "authored" / "prepare" / (name + ".json");
    scenario = parseJson(bytes(path));
    if (recorded) {
      // Apply only the preparation divergence documented in scene_golden/README.md.
      const std::string world = "$work/run/prepared-world.world";
      Json prepared = nullptr;
      for (auto& step : scenario.at("steps")) {
        if (step.at("operation") == "gazebo.prepare") {
          const auto authored = step.at("request").value("parameters", Json::object());
          if (authored.contains("chassis_robot_ids")) {
            step["response"] = Json{{"error", {{"type", "SceneError"},
                {"message", "scene parameter chassis_robot_ids is no longer accepted; the Gazebo chassis HOLD roster is dynamic"}}}};
          } else if (step.at("response").contains("value")) {
            auto& parameters = step.at("response").at("value").at("parameters");
            parameters.erase("chassis_robot_ids");
            prepared = step.at("effects").at("files").at(world);
            if (!authored.contains("required_components")) {
              parameters["required_components"] = Json::array({"vrpn"});
              auto contents = prepared.at("contents").get<std::string>();
              replace(contents, "<clock_rate_hz>", "<required_component>vrpn</required_component><clock_rate_hz>");
              prepared["contents"] = contents;
            }
          }
        }
        auto& files = step.at("effects").at("files");
        if (files.contains(world)) {
          if (prepared.is_null()) files.erase(world);
          else files[world] = prepared;
        }
      }
    }
    fs::create_directories(work); fs::create_directories(fixtures);
    bindings = {{"$work", work.string()}, {"$fixtures", fixtures.string()}};
    if (use_cli) bindings["$work/asset"] = (work / "Shared" / "Scenes" / "Fixture").string();
    for (const auto& path : scenario.at("setup").at("directories")) fs::create_directories(bind(path).get<std::string>());
    for (const auto& section : {"files", "executables"}) {
      if (!scenario.at("setup").contains(section)) continue;
      for (auto it = scenario.at("setup").at(section).begin(); it != scenario.at("setup").at(section).end(); ++it) {
        const auto path = fs::path(bind(Json(it.key())).get<std::string>());
        const auto value = bind(it.value());
        if (value.contains("symlink")) fs::create_symlink(value.at("symlink").get<std::string>(), path);
        else write(path, value.at("contents").get<std::string>(), value.at("mode").get<unsigned>());
      }
    }
    sourceDigest();
    if (use_cli) fs::create_directory(workspace());
  }

  Json bind(const Json& value) const {
    if (value.is_string()) {
      auto text = value.get<std::string>();
      std::vector<std::pair<std::string, std::string>> tokens(bindings.begin(), bindings.end());
      std::sort(tokens.begin(), tokens.end(), [](const auto& a, const auto& b) { return a.first.size() > b.first.size(); });
      for (const auto& token : tokens) replace(text, token.first, token.second);
      return text;
    }
    if (!value.is_structured()) return value;
    auto result = value.is_object() ? Json::object() : Json::array();
    for (auto it = value.begin(); it != value.end(); ++it) {
      if (value.is_object()) result[bind(Json(it.key())).get<std::string>()] = bind(it.value());
      else result.push_back(bind(it.value()));
    }
    return result;
  }
  fs::path asset() const { return bind(Json("$work/asset")).get<std::string>(); }
  bool retiredNativeAsset() const {
    return parseJson(bytes(asset() / "manifest.json")).at("simulators").at("gazebo").at("geometry") == "native";
  }
  Json preparationRequest() const {
    for (const auto& step : scenario.at("steps"))
      if (step.at("operation") == "gazebo.prepare") return bind(step.at("request"));
    throw std::runtime_error("No preparation request in fixture");
  }
  fs::path workspace() const { return work / "provider-workspace"; }
  fs::path cliOutput(bool document) const {
    return document ? workspace() / "native-scenes" / "experiment1" / "Fixture" : workspace() / "native-worlds" / "run1";
  }
  void sourceDigest() { bindings["$sourceSha256"] = sha256(asset() / "scene.yaml"); }
  void mutate(const Json& raw) {
    const auto request = bind(raw);
    const fs::path path = request.at("path").get<std::string>();
    if (request.contains("symlink")) {
      fs::remove(path); fs::create_symlink(request.at("symlink").get<std::string>(), path);
    } else {
      std::ofstream stream(path, std::ios::binary); stream << request.at("contents").get<std::string>();
      if (!stream) throw std::runtime_error("Cannot mutate preparation fixture");
    }
    sourceDigest();
  }
  void preparedDigest(const Json& step) {
    if (!step.at("response").contains("value")) return;
    const auto receipt = step.at("response").at("value");
    if (!receipt.is_object() || !receipt.contains("worldFile")) return;
    const auto& world = step.at("effects").at("files").at(receipt.at("worldFile").get<std::string>()).at("contents");
    bindings["$preparedSha256"] = digest(bind(world).get<std::string>());
  }
  void observe(const Json& raw, const Json& additional = Json::object()) const {
    auto expected = bind(raw.at("files"));
    expected.update(additional);
    Json actual = Json::object();
    for (const auto& entry : fs::recursive_directory_iterator(work)) {
      const auto path = entry.path();
      ASSERT_NE(path.filename().string().find(".scene-prepare-"), 0u) << "A staging directory survived";
      if (entry.is_symlink()) actual[path.string()] = Json{{"symlink", fs::read_symlink(path).string()}};
      else if (entry.is_regular_file()) {
        struct stat status{};
        ASSERT_EQ(::stat(path.c_str(), &status), 0);
        actual[path.string()] = Json{{"contents", bytes(path)}, {"mode", status.st_mode & 0777}};
      }
    }
    for (auto it = expected.begin(); it != expected.end(); ++it)
      if (it.value().contains("contents") && it.value().at("contents").is_null()) {
        EXPECT_FALSE(fs::exists(it.key())); actual[it.key()] = it.value();
      }
    same(actual, expected);
  }
};

class PrepareReplay : public ::testing::TestWithParam<std::string> {};

TEST_P(PrepareReplay, Library) {
  Fixture fixture(GetParam());
  if (fixture.retiredNativeAsset()) {
    // Preserve the sealed historical corpus. Its native admission contract is
    // retired; this is a rejection proof, not a replay of its former outputs.
    auto request = fixture.preparationRequest();
    request["configuration_revision"] = 1;
    request.erase("expected_revision_id");
    try { prepare(request); FAIL() << "Retired native Scene asset was admitted"; }
    catch (const SceneError& error) { EXPECT_STREQ(error.what(), "Scene asset geometry must be document"); }
    EXPECT_FALSE(fs::exists(fixture.work / "run/prepared-world.world"));
    return;
  }
  int calls = 0;
  for (const auto& step : fixture.scenario.at("steps")) {
    SCOPED_TRACE(step.at("operation").get<std::string>() + " " + std::to_string(calls));
    if (step.at("operation") == "file.mutate") fixture.mutate(step.at("request"));
    else {
      ASSERT_EQ(step.at("operation"), "gazebo.prepare");
      ++calls; fixture.preparedDigest(step);
      const auto response = fixture.bind(step.at("response"));
      try {
        const auto actual = prepare(fixture.bind(step.at("request")));
        ASSERT_TRUE(response.contains("value")) << response.dump();
        same(actual, response.at("value"));
      } catch (const SceneError& error) {
        ASSERT_TRUE(response.contains("error")) << error.what();
        EXPECT_EQ(response.at("error").at("type"), "SceneError");
        EXPECT_EQ(error.what(), response.at("error").at("message").get<std::string>());
      }
    }
    fixture.observe(step.at("effects"));
  }
  EXPECT_GT(calls, 0);
}

TEST_P(PrepareReplay, CliSelectionAndMaterialization) {
  Fixture fixture(GetParam(), true);
  if (fixture.retiredNativeAsset()) {
    Json context{{"scene", {{"asset", "Fixture"}, {"simulator", "gazebo"}}},
                 {"sessionId", "run1"}, {"openingRunId", "run1"}};
    for (const bool materialize : {false, true}) {
      std::vector<std::string> arguments{"--context-json", context.dump(), "--resource-root", fixture.work.string(),
          "--workspace-root", fixture.workspace().string(), "--socket-path", (fixture.workspace() / "world.sock").string(),
          "--target-id", "fixture", "--configuration-revision", "1"};
      if (materialize) arguments.insert(arguments.end(), {"--prepare-world", "--asset-directory", fixture.asset().string()});
      const auto result = cli(fixture.temporary.root, arguments);
      EXPECT_EQ(result.code, 1); EXPECT_TRUE(result.out.empty());
      EXPECT_EQ(result.err, "scene_prepare: Scene asset geometry must be document\n");
    }
    EXPECT_FALSE(fs::exists(fixture.workspace() / "native-worlds/run1/prepared-world.world"));
    return;
  }
  Json compiled_effect = Json::object();
  int calls = 0;
  for (const auto& step : fixture.scenario.at("steps")) {
    if (step.at("operation") == "file.mutate") { fixture.mutate(step.at("request")); continue; }
    SCOPED_TRACE(calls++);
    const auto request = fixture.bind(step.at("request"));
    const auto manifest = parseJson(bytes(fixture.asset() / "manifest.json"));
    const bool random = manifest.value("obstacleInput", Json::object()).value("mode", Json()) == "random";
    Json context{{"scene", {{"asset", "Fixture"}, {"simulator", "gazebo"}}},
                 {"experimentResourceId", "experiment1"}, {"openingRunId", "run1"}, {"sessionId", "run1"}};
    if (request.contains("parameters")) context["scene"]["parameters"] = request.at("parameters");
    std::vector<std::string> arguments{"--context-json", context.dump(), "--resource-root", fixture.work.string(),
                                       "--workspace-root", fixture.workspace().string(), "--socket-path", request.at("socket_path").get<std::string>(),
                                       "--target-id", request.at("target_id").get<std::string>(), "--configuration-revision", dumpJson(request.at("configuration_revision"))};
    // The script has neither an expected-revision flag nor a configurable byte
    // grant. These two raw request failures replay the selection path only.
    const bool inputs_only = request.value("expected_revision_id", Json("revision1")) != "revision1" || request.at("output_grant").at("max_bytes") == 1;
    if (!inputs_only) arguments.insert(arguments.end(), {"--prepare-world", "--asset-directory", fixture.asset().string()});
    const auto result = cli(fixture.temporary.root, arguments);
    if (!request.at("configuration_revision").is_number_integer()) {
      EXPECT_EQ(result.code, 2); EXPECT_TRUE(result.out.empty());
      EXPECT_EQ(result.err, "scene_prepare: --configuration-revision requires an integer\n");
    } else if (random) {
      // The CLI's 16 MiB grant reaches JSON validation for this 2 MB invalid
      // generator output, unlike the library case's smaller explicit grant.
      EXPECT_EQ(result.code, 1); EXPECT_TRUE(result.out.empty());
      EXPECT_EQ(result.err.find("scene_prepare: Invalid generator result: "), 0u);
      EXPECT_FALSE(fs::exists(fixture.workspace() / "native-worlds/run1/prepared-world.world"));
    } else if (inputs_only) {
      ASSERT_EQ(result.code, 0) << result.err;
      EXPECT_TRUE(result.err.empty());
      const auto value = parseJson(result.out);
      EXPECT_EQ(value.at("assetDirectory"), fixture.asset().string());
      EXPECT_EQ(value.at("worldFile"), (fixture.asset() / "source.world").string());
      EXPECT_EQ(value.at("sceneFile"), (fixture.asset() / "scene.yaml").string());
      EXPECT_EQ(value.at("workingFile"), ""); EXPECT_EQ(value.at("frozen"), true);
      const auto compiled = fixture.workspace() / "compiled-scenes/run1/scene-document.expanded.json";
      EXPECT_EQ(value.at("compiledSceneFile"), compiled.string());
      // This new finite expansion artifact is outside the historical Python
      // world-materialization corpus. Assert its exact empty-scene contract;
      // all other filesystem effects still replay that original corpus.
      compiled_effect[compiled.string()] = Json{{"contents", dumpJson(Json{
          {"schema", "xgc2.scene.compiled.v1"}, {"id", "fixture"}, {"frame", "world"},
          {"ground", {{"mode", "default_plane"}, {"z", 0.0}}},
          {"geometries", Json::array()}, {"models", Json::object()}, {"obstacles", Json::array()}})}, {"mode", 0600}};
    } else if (step.at("response").contains("error")) {
      EXPECT_EQ(result.code, 1); EXPECT_TRUE(result.out.empty());
      EXPECT_EQ(result.err, "scene_prepare: " + step.at("response").at("error").at("message").get<std::string>() + "\n");
    } else {
      ASSERT_EQ(result.code, 0) << result.err;
      EXPECT_TRUE(result.err.empty());
      fixture.bindings["$work/run/prepared-world.world"] = (fixture.workspace() / "native-worlds/run1/prepared-world.world").string();
      fixture.bindings["$work/run/scene-document.yaml"] = (fixture.workspace() / "native-worlds/run1/scene-document.yaml").string();
      fixture.preparedDigest(step);
      auto expected = fixture.bind(step.at("response").at("value"));
      expected["simulator"] = "gazebo"; expected["worldAttachment"] = true;
      expected["generationParametersJson"] = "{}";
      expected["sceneDocumentFile"] = manifest.value("sceneDocument", Json::object()).value("file", "");
      same(parseJson(result.out), expected);
    }
    fixture.observe(step.at("effects"), compiled_effect);
  }
  EXPECT_GT(calls, 0);
}

INSTANTIATE_TEST_SUITE_P(Golden, PrepareReplay, ::testing::ValuesIn(scenarios()),
                        [](const auto& info) { auto name = info.param; std::replace(name.begin(), name.end(), '-', '_'); return name; });

TEST(Prepare, BoundedReadsCheckedPathsAndAtomicReplacement) {
  Fixture fixture("native-world");
  const auto file = fixture.work / "bounded";
  write(file, "abcd");
  EXPECT_EQ(read(file, 4), "abcd");
  EXPECT_THROW(read(file, 3), SceneError);
  write(file, ""); EXPECT_THROW(read(file, 4), SceneError);
  EXPECT_THROW(checkedPath("relative", fixture.work), SceneError);
  EXPECT_THROW(checkedPath(fixture.work.string(), fixture.work), SceneError);
  EXPECT_THROW(checkedPath(file.string(), fixture.work, true), SceneError);
  const auto outside = fixture.temporary.root / "outside";
  write(outside, "owner");
  fs::remove(file); fs::create_symlink(outside, file);
  EXPECT_THROW(checkedPath(file.string(), fixture.work), SceneError);
  atomic(file, "replacement");
  EXPECT_FALSE(fs::is_symlink(file)); EXPECT_EQ(bytes(file), "replacement"); EXPECT_EQ(bytes(outside), "owner");
  struct stat status{}; ASSERT_EQ(::stat(file.c_str(), &status), 0); EXPECT_EQ(status.st_mode & 0777, 0600);
  fs::create_directory(fixture.work / "blocked");
  EXPECT_THROW(atomic(fixture.work / "blocked", "bytes"), std::system_error);
  for (const auto& entry : fs::directory_iterator(fixture.work)) EXPECT_NE(entry.path().filename().string().find(".blocked."), 0u);
}

TEST(Prepare, WorldXmlMatchesElementTreeTextAttributesAndEmptyElements) {
  Fixture fixture("native-world");
  const auto source = fixture.work / "xml.world";
  write(source, "<?xml version=\"1.0\"?>\r\n<sdf version='1.6'>\r\n<world name='a&amp;b'> text<!--ignored--><?discard data?><model a='x&#9;y\n&amp;&quot;' z='last'><![CDATA[<&>]]><link/></model> tail\r\n</world></sdf>");
  auto request = fixture.bind(fixture.scenario.at("steps").at(0).at("request"));
  request.erase("asset_directory"); request["world_file"] = source.string();
  const auto receipt = prepare(request);
  const std::string expected = "<?xml version='1.0' encoding='utf-8'?>\n<sdf version=\"1.6\">\n<world name=\"a&amp;b\"> text<model a=\"x&#09;y &amp;&quot;\" z=\"last\">&lt;&amp;&gt;<link /></model> tail\n"
      "<plugin name=\"xgc2_simulation\" filename=\"libxgc2_simulation_world.so\"><socket_path>" + (fixture.work / "run/world.sock").string() +
      "</socket_path><target_id>fixture</target_id><resource_root>" + fixture.work.string() +
      "</resource_root><configuration_revision>4</configuration_revision><required_component>vrpn</required_component>"
      "<clock_rate_hz>0</clock_rate_hz></plugin></world></sdf>";
  EXPECT_EQ(bytes(receipt.at("worldFile").get<std::string>()), expected);
  EXPECT_EQ(receipt.at("preparedWorldSha256"), digest(expected));
}

TEST(Prepare, InvalidWorldsNeverReplacePreparedArtifacts) {
  Fixture fixture("native-world");
  auto request = fixture.bind(fixture.scenario.at("steps").at(0).at("request"));
  request.erase("asset_directory"); request["world_file"] = (fixture.asset() / "source.world").string();
  const auto original = prepare(request);
  const auto target = fs::path(original.at("worldFile").get<std::string>());
  const auto committed = bytes(target);
  const auto source = fixture.work / "xml.world";
  request.erase("asset_directory"); request["world_file"] = source.string();
  const std::vector<std::pair<std::string, std::string>> cases{
      {"<sdf><world>", "Invalid Gazebo world XML"}, {"<sdf><world></sdf>", "Invalid Gazebo world XML"},
      {"<sdf><world a='1' a='2'/></sdf>", "Invalid Gazebo world XML"}, {"<sdf><world>&unknown;</world></sdf>", "Invalid Gazebo world XML"},
      {"<sdf><world>&#0;</world></sdf>", "Invalid Gazebo world XML"}, {"<sdf><world/><world/></sdf>", "A single-world SDF asset is required"},
      {"<?xml?><sdf><world/></sdf>", "Invalid Gazebo world XML"}, {"<?XML version='1.0'?><sdf><world/></sdf>", "Invalid Gazebo world XML"},
      {"<world/>", "A single-world SDF asset is required"}, {"<!DOCTYPE sdf><sdf><world/></sdf>", "World assets cannot declare XML entities"},
      {"<sdf><world><!-- <!entity forbidden --> </world></sdf>", "World assets cannot declare XML entities"},
      {"<sdf><world><plugin filename='libxgc2_simulation_world.so'/></world></sdf>", "The input world already embeds a platform control owner"},
      {"<sdf><world><plugin filename='libxgc2_scene_authoring_world.so'/></world></sdf>", "The input world already embeds a platform control owner"}};
  for (const auto& item : cases) {
    SCOPED_TRACE(item.first); write(source, item.first);
    try { prepare(request); FAIL() << "Invalid world was accepted"; }
    catch (const SceneError& error) { EXPECT_EQ(error.what(), item.second); }
    EXPECT_EQ(bytes(target), committed);
  }
}

TEST(Prepare, NativeAssetsAndMissingDocumentsAreRejected) {
  Fixture fixture("native-world", true);
  auto manifest = parseJson(bytes(fixture.asset() / "manifest.json"));
  auto request = fixture.bind(fixture.scenario.at("steps").at(0).at("request"));
  Json context{{"scene", {{"asset", "Fixture"}, {"simulator", "gazebo"}}}, {"sessionId", "session1"}};
  const auto reject = [&](const std::string& prepare_message, const std::string& inputs_message) {
    try { prepare(request); FAIL() << "Invalid Scene asset was admitted"; }
    catch (const SceneError& error) { EXPECT_EQ(error.what(), prepare_message); }
    try { inputs(context, fixture.work, fixture.workspace()); FAIL() << "Invalid Scene asset was selected"; }
    catch (const SceneError& error) { EXPECT_EQ(error.what(), inputs_message); }
    EXPECT_TRUE(fs::is_empty(fixture.workspace()));
    EXPECT_FALSE(fs::exists(fixture.work / "run/prepared-world.world"));
  };
  reject("Scene asset geometry must be document", "Scene asset geometry must be document");
  context["scene"]["simulator"] = "none";
  EXPECT_THROW(inputs(context, fixture.work, fixture.workspace()), SceneError);
  context["scene"]["simulator"] = "gazebo";
  manifest["simulators"]["gazebo"]["geometry"] = "document";
  manifest["sceneDocument"] = Json{{"file", ""}, {"sha256", ""}};
  write(fixture.asset() / "manifest.json", manifest.dump());
  reject("Required scene attachment is absent", "Required scene attachment is absent");
  context["scene"]["simulator"] = "none";
  EXPECT_THROW(inputs(context, fixture.work, fixture.workspace()), SceneError);
  EXPECT_EQ(attachment(fixture.asset(), manifest["sceneDocument"]), "");
  EXPECT_THROW(attachment(fixture.asset(), manifest["sceneDocument"], true), SceneError);
}

TEST(Prepare, DocumentAdmissionPreservesValidationAndAtomicWorldWrites) {
  Fixture fixture("document-and-media");
  auto request = fixture.preparationRequest();
  const auto original = prepare(request);
  EXPECT_EQ(original.at("projectGeometry"), true);
  EXPECT_FALSE(original.at("sceneFile").get<std::string>().empty());
  const auto target = fs::path(original.at("worldFile").get<std::string>());
  const auto committed = bytes(target);
  const auto reject = [&](const Json& value, const std::string& expected) {
    try { prepare(value); FAIL() << "Invalid document request was accepted"; }
    catch (const SceneError& error) { EXPECT_EQ(error.what(), expected); }
    EXPECT_EQ(bytes(target), committed);
  };
  auto bad = request; bad["expected_revision_id"] = "wrong";
  reject(bad, "Scene asset revision conflicts with the requested reference");
  bad = request; bad["parameters"] = Json{{"typo", 1}};
  reject(bad, "Invalid fields: unknown=['typo'], missing=[]");
  bad = request; bad["parameters"] = Json{{"maxStepSize", true}};
  reject(bad, "maxStepSize must be a finite positive number");
  bad = request; bad["configuration_revision"] = 4.0;
  reject(bad, "A versioned simulation prepare request is required");
  bad = request; bad["output_grant"]["max_bytes"] = 1;
  reject(bad, "Prepared world exhausts the output grant");
  bad = request; bad["parameters"] = Json{{"chassis_robot_ids", Json::array({"ugv1"})}};
  reject(bad, "scene parameter chassis_robot_ids is no longer accepted; the Gazebo chassis HOLD roster is dynamic");
  for (const auto& components : {Json::array(), Json::array({"camera"})}) {
    auto selected = request; selected["parameters"] = Json{{"required_components", components}};
    EXPECT_EQ(prepare(selected).at("parameters").at("required_components"), components);
  }
  prepare(request);
  auto manifest = parseJson(bytes(fixture.asset() / "manifest.json"));
  const auto pristine = manifest;
  for (const auto* key : {"sceneDocument", "gazeboWorld"}) {
    manifest = pristine; manifest[key]["sha256"] = std::string(64, '0');
    write(fixture.asset() / "manifest.json", manifest.dump());
    reject(request, "Scene attachment content does not match its manifest SHA256");
  }
  write(fixture.asset() / "manifest.json", pristine.dump());
  const auto outside = fixture.temporary.root / "outside.world";
  write(outside, bytes(fixture.asset() / "source.world"));
  fs::remove(fixture.asset() / "source.world"); fs::create_symlink(outside, fixture.asset() / "source.world");
  reject(request, "Resource path is outside the granted root");
}

TEST(Prepare, WorkspaceSelectionAndCliContextValidation) {
  Fixture fixture("document-and-media", true);
  auto manifest = parseJson(bytes(fixture.asset() / "manifest.json"));
  manifest["simulators"]["xsim"] = Json{{"geometry", "document"}};
  write(fixture.asset() / "manifest.json", manifest.dump());
  Json context{{"scene", {{"asset", "Fixture"}, {"simulator", "xsim"}, {"parameters", {{"generation", {{"seed", 1.0}}}}}}},
               {"experimentResourceId", "experiment1"}, {"sessionId", "session1"}};
  const auto selected = inputs(context, fixture.work, fixture.workspace());
  auto mixed = manifest;
  mixed["simulators"]["gazebo"]["geometry"] = "native";
  write(fixture.asset() / "manifest.json", mixed.dump());
  EXPECT_THROW(inputs(context, fixture.work, fixture.workspace()), SceneError);
  auto physical_only = context; physical_only["scene"]["simulator"] = "none";
  EXPECT_THROW(inputs(physical_only, fixture.work, fixture.workspace()), SceneError);
  mixed["simulators"] = Json::object();
  write(fixture.asset() / "manifest.json", mixed.dump());
  EXPECT_EQ(inputs(physical_only, fixture.work, fixture.workspace()).at("sceneFile"), selected.at("sceneFile"));
  EXPECT_THROW(inputs(context, fixture.work, fixture.workspace()), SceneError);
  write(fixture.asset() / "manifest.json", manifest.dump());
  EXPECT_EQ(selected.at("workingFile"), (fixture.cliOutput(true) / "scene-document.yaml").string());
  EXPECT_EQ(selected.at("generationParametersJson"), "{\"seed\":1.0}");
  EXPECT_EQ(selected.at("frozen"), false); EXPECT_EQ(selected.at("worldFile"), "");
  context["scene"]["parameters"]["lidar"] = Json{{"scope", "global"}, {"settings", {{"enabled", true}, {"mode", "penetrating"}, {"acceleration", "gpu"}}}};
  EXPECT_EQ(inputs(context, fixture.work, fixture.workspace()), selected);
  context["scene"]["parameters"].erase("lidar");
  auto physical_context = context;
  physical_context["scene"]["simulator"] = "none";
  auto physical_selected = selected;
  physical_selected["simulator"] = "none";
  EXPECT_EQ(inputs(physical_context, fixture.work, fixture.workspace()), physical_selected);

  EXPECT_FALSE(fs::exists(selected.at("workingFile").get<std::string>()));
  context["scene"]["parameters"]["frozen"] = false;
  EXPECT_EQ(inputs(context, fixture.work, fixture.workspace()), selected);
  context["scene"]["parameters"]["frozen"] = true;
  const auto locked = inputs(context, fixture.work, fixture.workspace());
  EXPECT_EQ(locked.at("workingFile"), "");
  EXPECT_EQ(locked.at("frozen"), true);
  EXPECT_FALSE(locked.at("parameters").contains("frozen"));
  context["scene"]["parameters"]["frozen"] = "false";
  EXPECT_THROW(inputs(context, fixture.work, fixture.workspace()), SceneError);
  context["scene"]["parameters"].erase("frozen");
  const auto result = cli(fixture.temporary.root, {"--context-json=" + context.dump()},
                          {{"XGC_USER_FILES_DIR", fixture.work.string()}, {"XGC_JOB_ARTIFACT_ROOT", fixture.workspace().string()}});
  ASSERT_EQ(result.code, 0) << result.err; same(parseJson(result.out), selected);
  fs::remove_all(fixture.workspace() / "native-scenes");
  const auto outside = fixture.temporary.root / "other-provider"; fs::create_directory(outside);
  fs::create_directory_symlink(outside, fixture.workspace() / "native-scenes");
  EXPECT_THROW(inputs(context, fixture.work, fixture.workspace()), SceneError);
  EXPECT_TRUE(fs::is_empty(outside));
  for (const auto& raw : {std::string(65537, 'x'), std::string("{\"scene\":{},\"scene\":{}}"), std::string("{\"scene\":NaN}")}) {
    const auto failed = cli(fixture.temporary.root, {"--context-json", raw});
    EXPECT_EQ(failed.code, 1); EXPECT_TRUE(failed.out.empty());
    const auto message = raw.size() > 65536 ? "Scene context exceeds 64 KiB" : raw.find("NaN") != std::string::npos ? "Nonfinite JSON" : "Duplicate scene field scene";
    EXPECT_EQ(failed.err, std::string("scene_prepare: ") + message + "\n");
  }
}

TEST(Prepare, LegacyChassisParameterIsRejectedForXsimSelection) {
  Fixture fixture("document-and-media", true);
  auto manifest = parseJson(bytes(fixture.asset() / "manifest.json"));
  manifest["simulators"]["xsim"] = Json{{"geometry", "document"}};
  write(fixture.asset() / "manifest.json", manifest.dump());
  Json context{{"scene", {{"asset", "Fixture"}, {"simulator", "xsim"}, {"parameters", {{"chassis_robot_ids", Json::array()}}}}},
               {"experimentResourceId", "experiment1"}};
  const std::string message = "scene parameter chassis_robot_ids is no longer accepted; the Gazebo chassis HOLD roster is dynamic";
  try { inputs(context, fixture.work, fixture.workspace()); FAIL() << "Legacy scene parameter was accepted"; }
  catch (const SceneError& error) { EXPECT_EQ(error.what(), message); }
  const auto result = cli(fixture.temporary.root, {"--context-json", context.dump(),
                          "--resource-root", fixture.work.string(), "--workspace-root", fixture.workspace().string()});
  EXPECT_EQ(result.code, 1); EXPECT_TRUE(result.out.empty());
  EXPECT_EQ(result.err, "scene_prepare: " + message + "\n");
  EXPECT_TRUE(fs::is_empty(fixture.workspace()));
}

TEST(Prepare, RandomArtifactsAreStagedAndExistingResultsAreReused) {
  Fixture fixture("document-and-media", true);
  const auto generator = fixture.fixtures / "generator.py", counter = fixture.fixtures / "counter";
  write(generator, "#!/usr/bin/env python3.8\nimport json\nfrom pathlib import Path\np = Path('" + counter.string() + "')\np.write_text(p.read_text() + 'run\\n' if p.exists() else 'run\\n')\nprint(json.dumps({'scene': {'schema': 'xgc2.scene.v1', 'id': 'generated', 'frame': 'world', 'obstacles': []}, 'producer': 'fixture'}))\n", 0755);
  Json source{{"schema", "xgc2.scene-source.v1"}, {"format", "geometry"}, {"mode", "random"},
              {"generator", {{"command", Json::array({generator.string()})}, {"parameters", Json::object()}}}};
  write(fixture.asset() / "scene.yaml", dumpYaml(source));
  auto manifest = parseJson(bytes(fixture.asset() / "manifest.json"));
  manifest["sceneDocument"]["sha256"] = sha256(fixture.asset() / "scene.yaml");
  manifest["obstacleInput"] = Json{{"mode", "random"}, {"format", "geometry"}};
  write(fixture.asset() / "manifest.json", manifest.dump());
  auto request = fixture.bind(fixture.scenario.at("steps").back().at("request"));
  request["run_id"] = "run1"; request["editable"] = false;
  const auto receipt = prepare(request);
  EXPECT_TRUE(receipt.at("frozen").get<bool>());
  EXPECT_EQ(receipt.at("sceneFile"), (fixture.work / "run/scene-document.yaml").string());
  EXPECT_EQ(receipt.at("writes").size(), 3u);
  EXPECT_EQ(load(receipt.at("sceneFile").get<std::string>()).first.at("id"), "generated");
  EXPECT_TRUE(fs::exists(fixture.work / "run/scene-document.generation.yaml"));
  for (const auto& entry : fs::directory_iterator(fixture.work / "run")) EXPECT_NE(entry.path().filename().string().find(".scene-prepare-"), 0u);
  EXPECT_EQ(bytes(counter), "run\n");
  EXPECT_EQ(prepare(request).at("writes").size(), 1u);
  EXPECT_EQ(bytes(counter), "run\n");
  const Json context{{"scene", {{"asset", "Fixture"}, {"simulator", "gazebo"}}},
                     {"experimentResourceId", "experiment1"}, {"sessionId", "session1"}};
  const auto selected = inputs(context, fixture.work, fixture.workspace());
  auto mixed = manifest;
  mixed["simulators"]["gazebo"]["geometry"] = "native";
  write(fixture.asset() / "manifest.json", mixed.dump());
  EXPECT_THROW(inputs(context, fixture.work, fixture.workspace()), SceneError);
  auto physical_only = context; physical_only["scene"]["simulator"] = "none";
  EXPECT_THROW(inputs(physical_only, fixture.work, fixture.workspace()), SceneError);
  mixed["simulators"] = Json::object();
  write(fixture.asset() / "manifest.json", mixed.dump());
  EXPECT_EQ(inputs(physical_only, fixture.work, fixture.workspace()).at("sceneFile"), selected.at("sceneFile"));
  EXPECT_THROW(inputs(context, fixture.work, fixture.workspace()), SceneError);
  write(fixture.asset() / "manifest.json", manifest.dump());
  EXPECT_EQ(selected.at("workingFile"), (fixture.cliOutput(true) / "session1" / "scene.yaml").string());
  EXPECT_EQ(selected.at("sceneFile"), selected.at("workingFile"));
  EXPECT_EQ(inputs(context, fixture.work, fixture.workspace()), selected);
  EXPECT_EQ(bytes(counter), "run\nrun\n");
}

TEST(Prepare, RandomInputsProduceOnePortableFlatAssetPerSession) {
  Fixture fixture("document-and-media", true);
  const auto generator = fixture.fixtures / "generator.py", counter = fixture.fixtures / "counter";
  write(generator, "#!/usr/bin/env python3.8\nfrom pathlib import Path\nimport json\np = Path('" + counter.string() + "')\np.write_text(p.read_text() + 'run\\n' if p.exists() else 'run\\n')\nprint(json.dumps({'scene': {'schema': 'xgc2.scene.v1', 'id': 'expanded', 'frame': 'world', 'obstacles': []}}))\n", 0755);
  auto manifest = parseJson(bytes(fixture.asset() / "manifest.json"));
  fs::remove(fixture.asset() / "scene.yaml");
  const std::string filename = "custom-obstacles.yaml";
  const auto source = dumpYaml(Json{{"schema", "xgc2.scene-source.v1"}, {"format", "geometry"}, {"mode", "random"},
      {"generator", {{"command", Json::array({generator.string()})}, {"parameters", Json::object()}}}});
  write(fixture.asset() / filename, source);
  manifest["sceneDocument"] = Json{{"file", filename}, {"sha256", digest(source)}};
  manifest["obstacleInput"] = Json{{"mode", "random"}, {"format", "geometry"}, {"generator", "fixture"}, {"parameters", {{"seed", 1}}}};
  manifest["simulators"]["xsim"] = Json{{"geometry", "document"}};
  const std::string cloud("PCD\0unchanged\n", 14);
  write(fixture.asset() / "fixed-cloud.pcd", cloud);
  manifest["pointCloud"] = Json{{"file", "fixed-cloud.pcd"}, {"sha256", digest(cloud)}, {"format", "pcd"}, {"frame", "world"}};
  write(fixture.asset() / "manifest.json", dumpJson(manifest));
  std::map<std::string, std::string> sealed;
  for (const auto& file : fs::directory_iterator(fixture.asset())) sealed[file.path().filename().string()] = bytes(file.path());
  Json context{{"scene", {{"asset", "Fixture"}, {"simulator", "gazebo"}}},
               {"experimentResourceId", "experiment1"}, {"sessionId", "session1"}};
  const auto output = fixture.cliOutput(true) / "session1";
  fs::create_directories(output);
  // An existing expansion plus its ordinary generation sidecar needs only the
  // runtime asset envelope; it must not invoke the generator again.
  resolve(fixture.asset() / filename, output / filename, Json::object(), 8 * 1024 * 1024);
  const auto selected = inputs(context, fixture.work, fixture.workspace());
  ASSERT_EQ(selected.at("assetDirectory"), output.string());
  EXPECT_EQ(selected.at("sceneFile"), (output / filename).string());
  EXPECT_EQ(selected.at("workingFile"), selected.at("sceneFile"));
  EXPECT_EQ(selected.at("sceneDocumentFile"), filename);
  EXPECT_FALSE(selected.at("frozen").get<bool>());
  EXPECT_EQ(selected.at("pointCloud"), manifest.at("pointCloud"));
  EXPECT_EQ(bytes(output / "fixed-cloud.pcd"), cloud);
  EXPECT_FALSE(fs::exists(output / "scene-document.yaml"));
  EXPECT_EQ(bytes(counter), "run\n");
  const auto runtime = parseJson(bytes(output / "manifest.json"));
  EXPECT_EQ(runtime.at("obstacleInput"), (Json{{"mode", "fixed"}, {"format", "geometry"}}));
  EXPECT_EQ(runtime.at("sceneDocument").at("sha256"), sha256(output / filename));
  for (const auto& file : sealed) if (file.first != "manifest.json" && file.first != filename)
    EXPECT_EQ(bytes(output / file.first), file.second);
  const auto published = bytes(output / "manifest.json");
  EXPECT_EQ(inputs(context, fixture.work, fixture.workspace()), selected);
  EXPECT_EQ(bytes(output / "manifest.json"), published);
  EXPECT_EQ(bytes(counter), "run\n");
  context["sessionId"] = "session2";
  const auto other = inputs(context, fixture.work, fixture.workspace());
  EXPECT_NE(other.at("assetDirectory"), selected.at("assetDirectory"));
  EXPECT_EQ(bytes(counter), "run\nrun\n");
  for (const auto& file : sealed) EXPECT_EQ(bytes(fixture.asset() / file.first), file.second);
  write(fixture.asset() / "custom-obstacles.generation.yaml", "sealed attachment");
  context["sessionId"] = "session1";
  EXPECT_THROW(inputs(context, fixture.work, fixture.workspace()), SceneError);
  EXPECT_EQ(bytes(output / "manifest.json"), published);
  EXPECT_EQ(bytes(counter), "run\nrun\n");
  fs::remove(fixture.asset() / "custom-obstacles.generation.yaml");

  const auto moved_root = fixture.temporary.root / "relocated", moved = moved_root / "asset", prepared = moved_root / "prepared";
  fs::create_directories(prepared);
  fs::rename(output, moved);
  fs::remove_all(fixture.asset()); fs::remove(generator);
  Json request{{"schema", kPrepareSchema}, {"configuration_revision", 1}, {"asset_directory", moved.string()},
      {"resource_root", moved_root.string()}, {"socket_path", (prepared / "world.sock").string()}, {"target_id", "fixture"},
      {"output_grant", {{"directory", prepared.string()}, {"max_bytes", 16 * 1024 * 1024}}}};
  const auto receipt = prepare(request);
  EXPECT_EQ(receipt.at("sceneFile"), (moved / filename).string());
  EXPECT_EQ(load(receipt.at("sceneFile").get<std::string>()).first.at("id"), "expanded");
  EXPECT_EQ(bytes(counter), "run\nrun\n");
  // The same materialized asset also works with the actual provider template.
  auto template_manifest = runtime;
  template_manifest.erase("gazeboWorld");
  write(moved / "manifest.json", dumpJson(template_manifest));
  EXPECT_FALSE(prepare(request).at("worldFile").get<std::string>().empty());
  const auto world = bytes(prepared / "prepared-world.world");
  write(moved / filename, "tampered");
  EXPECT_THROW(prepare(request), SceneError);
  EXPECT_EQ(bytes(prepared / "prepared-world.world"), world);
}

TEST(Prepare, RandomInputsCopyNestedFilesAndRejectSpecialEntries) {
  Fixture fixture("document-and-media", true);
  auto manifest = parseJson(bytes(fixture.asset() / "manifest.json"));
  manifest["obstacleInput"] = Json{{"mode", "random"}, {"format", "geometry"}};
  write(fixture.asset() / "manifest.json", dumpJson(manifest));
  const Json context{{"scene", {{"asset", "Fixture"}, {"simulator", "gazebo"}}},
      {"experimentResourceId", "experiment1"}, {"sessionId", "session1"}};
  const auto nested = fixture.asset() / "models" / "barrier";
  fs::create_directories(nested);
  write(nested / "model.sdf", "<sdf/>");
  const auto selected = inputs(context, fixture.work, fixture.workspace());
  EXPECT_EQ(bytes(fs::path(selected.at("assetDirectory").get<std::string>()) / "models" / "barrier" / "model.sdf"), "<sdf/>");
  const auto bad = fixture.asset() / "unsupported";
  fs::create_symlink(fixture.asset() / "scene.yaml", bad);
  EXPECT_THROW(inputs(context, fixture.work, fixture.workspace()), SceneError);
  fs::remove(bad); ASSERT_EQ(::mkfifo(bad.c_str(), 0600), 0);
  EXPECT_THROW(inputs(context, fixture.work, fixture.workspace()), SceneError);
}

TEST(Prepare, CliConsumesRelocatedRuntimeAssetWithoutResolvingTheLibrary) {
  Fixture fixture("document-and-media", true);
  const auto generator = fixture.fixtures / "generator.py", counter = fixture.fixtures / "counter";
  write(generator, "#!/usr/bin/env python3.8\nfrom pathlib import Path\nimport json\np = Path('" + counter.string() + "')\np.write_text(p.read_text() + 'run\\n' if p.exists() else 'run\\n')\nprint(json.dumps({'scene': {'schema': 'xgc2.scene.v1', 'id': 'portable', 'frame': 'world', 'obstacles': []}}))\n", 0755);
  auto manifest = parseJson(bytes(fixture.asset() / "manifest.json"));
  const auto source = dumpYaml(Json{{"schema", "xgc2.scene-source.v1"}, {"format", "geometry"}, {"mode", "random"},
      {"generator", {{"command", Json::array({generator.string()})}, {"parameters", Json::object()}}}});
  fs::remove(fixture.asset() / "scene.yaml");
  write(fixture.asset() / "portable.yaml", source);
  manifest["sceneDocument"] = Json{{"file", "portable.yaml"}, {"sha256", digest(source)}};
  manifest["obstacleInput"] = Json{{"mode", "random"}, {"format", "geometry"}, {"generator", "fixture"}};
  manifest.erase("gazeboWorld");
  const std::string cloud("PCD\0bytes", 9);
  write(fixture.asset() / "points.pcd", cloud);
  manifest["pointCloud"] = Json{{"file", "points.pcd"}, {"sha256", digest(cloud)}, {"format", "pcd"}, {"frame", "world"}};
  write(fixture.asset() / "manifest.json", dumpJson(manifest));
  Json context{{"scene", {{"asset", "Fixture"}, {"simulator", "gazebo"}, {"parameters", {{"frozen", true}, {"lidar", Json::object()}}}}},
      {"experimentResourceId", "experiment1"}, {"sessionId", "session1"}, {"openingRunId", "opening1"}};
  const auto expanded = cli(fixture.temporary.root, {"--context-json", context.dump(), "--resource-root", fixture.work.string(),
      "--workspace-root", fixture.workspace().string()});
  ASSERT_EQ(expanded.code, 0) << expanded.err;
  const auto selected = parseJson(expanded.out);
  EXPECT_EQ(bytes(counter), "run\n");
  const auto grant = fixture.temporary.root / "relocated", asset = grant / "asset", workspace = grant / "workspace";
  fs::create_directories(workspace);
  fs::rename(selected.at("assetDirectory").get<std::string>(), asset);
  fs::remove_all(fixture.asset()); fs::remove(generator);
  context["scene"]["asset"] = "Library no longer exists";
  auto arguments = [&](const Json& value) { return std::vector<std::string>{"--context-json", value.dump(), "--prepare-world",
      "--asset-directory", asset.string(), "--resource-root", grant.string(), "--workspace-root", workspace.string(),
      "--socket-path", (workspace / "world.sock").string(), "--target-id", "fixture", "--configuration-revision", "1"}; };
  const auto prepared = cli(fixture.temporary.root, arguments(context));
  ASSERT_EQ(prepared.code, 0) << prepared.err;
  const auto value = parseJson(prepared.out);
  EXPECT_EQ(value.at("assetDirectory"), asset.string());
  EXPECT_EQ(value.at("sceneFile"), (asset / "portable.yaml").string());
  EXPECT_EQ(value.at("sceneDocumentFile"), "portable.yaml");
  EXPECT_EQ(value.at("pointCloud"), manifest.at("pointCloud"));
  EXPECT_EQ(bytes(asset / "points.pcd"), cloud);
  EXPECT_EQ(value.at("worldFile"), (workspace / "native-worlds/opening1/prepared-world.world").string());
  EXPECT_EQ(value.at("frozen"), true);
  EXPECT_FALSE(value.at("parameters").contains("frozen"));
  EXPECT_FALSE(value.at("parameters").contains("lidar"));
  const auto repeated = cli(fixture.temporary.root, arguments(context));
  ASSERT_EQ(repeated.code, 0) << repeated.err;
  EXPECT_EQ(parseJson(repeated.out), value);
  EXPECT_EQ(bytes(counter), "run\n");
  context["scene"]["parameters"]["frozen"] = false;
  const auto editable = cli(fixture.temporary.root, arguments(context));
  ASSERT_EQ(editable.code, 0) << editable.err;
  EXPECT_EQ(parseJson(editable.out).at("frozen"), false);
  EXPECT_EQ(parseJson(editable.out).at("sceneDocumentFile"), "portable.yaml");
  EXPECT_EQ(bytes(counter), "run\n");
  write(asset / "portable.yaml", "tampered");
  const auto rejected = cli(fixture.temporary.root, arguments(context));
  EXPECT_EQ(rejected.code, 1); EXPECT_TRUE(rejected.out.empty());
  EXPECT_EQ(rejected.err, "scene_prepare: Scene attachment content does not match its manifest SHA256\n");
}

TEST(Prepare, CliRequiresExplicitAssetGrantAndGazeboModeForPreparation) {
  Fixture fixture("native-world", true);
  Json context{{"scene", {{"asset", "Fixture"}, {"simulator", "gazebo"}}}, {"openingRunId", "run1"}};
  auto arguments = [&](bool asset, const fs::path& root) {
    std::vector<std::string> result{"--context-json", context.dump(), "--prepare-world", "--resource-root", root.string(),
        "--workspace-root", fixture.workspace().string(), "--socket-path", (fixture.workspace() / "world.sock").string(),
        "--target-id", "fixture", "--configuration-revision", "1"};
    if (asset) result.insert(result.end(), {"--asset-directory", fixture.asset().string()});
    return result;
  };
  const auto missing = cli(fixture.temporary.root, arguments(false, fixture.work));
  EXPECT_EQ(missing.code, 1); EXPECT_TRUE(missing.out.empty());
  EXPECT_EQ(missing.err, "scene_prepare: --prepare-world requires --asset-directory\n");
  const auto outside = fixture.temporary.root / "other-grant";
  fs::create_directory(outside);
  const auto denied = cli(fixture.temporary.root, arguments(true, outside));
  EXPECT_EQ(denied.code, 1); EXPECT_TRUE(denied.out.empty());
  EXPECT_EQ(denied.err, "scene_prepare: Resource path is outside the granted root\n");
  context["scene"]["simulator"] = "xsim";
  const auto unsupported = cli(fixture.temporary.root, arguments(true, fixture.work));
  EXPECT_EQ(unsupported.code, 1); EXPECT_TRUE(unsupported.out.empty());
  EXPECT_EQ(unsupported.err, "scene_prepare: --prepare-world requires the gazebo simulator\n");
  const auto expand = cli(fixture.temporary.root, {"--context-json", context.dump(), "--asset-directory", fixture.asset().string(),
      "--resource-root", fixture.work.string(), "--workspace-root", fixture.workspace().string()});
  EXPECT_EQ(expand.code, 1); EXPECT_TRUE(expand.out.empty());
  EXPECT_EQ(expand.err, "scene_prepare: --asset-directory requires --prepare-world\n");
  context["scene"]["simulator"] = "gazebo";
  const auto no_root = cli(fixture.temporary.root, arguments(true, ""));
  EXPECT_EQ(no_root.code, 1); EXPECT_TRUE(no_root.out.empty());
  EXPECT_EQ(no_root.err, "scene_prepare: An existing absolute resource_root is required\n");
  context["scene"]["parameters"] = Json{{"frozen", "false"}};
  const auto invalid_frozen = cli(fixture.temporary.root, arguments(true, fixture.work));
  EXPECT_EQ(invalid_frozen.code, 1); EXPECT_TRUE(invalid_frozen.out.empty());
  EXPECT_EQ(invalid_frozen.err, "scene_prepare: Scene parameter frozen must be a boolean\n");
  context["scene"].erase("parameters"); context.erase("openingRunId");
  const auto no_run = cli(fixture.temporary.root, arguments(true, fixture.work));
  EXPECT_EQ(no_run.code, 1); EXPECT_TRUE(no_run.out.empty());
  EXPECT_EQ(no_run.err, "scene_prepare: World materialization requires the actual Session opening Run identity\n");
}

}  // namespace
}  // namespace xgc_scene
