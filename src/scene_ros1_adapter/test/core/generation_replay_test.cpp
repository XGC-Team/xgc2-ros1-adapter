#include "xgc_scene/generation.hpp"

#include "xgc_scene/document.hpp"
#include "xgc_scene/json_io.hpp"
#include "xgc_scene/yaml_io.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <future>
#include <limits>
#include <set>
#include <sys/prctl.h>
#include <system_error>
#include <unistd.h>

namespace xgc_scene {
namespace {

namespace fs = std::filesystem;
constexpr char kPython[] = "/usr/bin/python3.8";
constexpr char kResult[] = R"({"seed":7,"scene":{"schema":"xgc2.scene.v1","id":"generated","frame":"world","obstacles":[]}})";

std::string read(const fs::path& path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) throw std::runtime_error("Cannot read " + path.string());
  return {std::istreambuf_iterator<char>(stream), std::istreambuf_iterator<char>()};
}

void write(const fs::path& path, const std::string& body, unsigned mode = 0644) {
  fs::create_directories(path.parent_path());
  std::ofstream stream(path, std::ios::binary);
  stream << body;
  stream.close();
  if (!stream) throw std::runtime_error("Cannot write " + path.string());
  fs::permissions(path, static_cast<fs::perms>(mode), fs::perm_options::replace);
}

std::size_t children() {
  std::size_t count = 0;
  // Include children of every thread of this test process, including zombies.
  for (const auto& task : fs::directory_iterator("/proc/self/task")) {
    std::ifstream stream(task.path() / "children");
    pid_t child;
    while (stream >> child) ++count;
  }
  return count;
}

class GenerationTest : public ::testing::Test {
 protected:
  fs::path root_;
  int original_subreaper_ = 0;

  void SetUp() override {
    std::string pattern = (fs::temp_directory_path() / "xgc-scene-generation-XXXXXX").string();
    const auto path = ::mkdtemp(pattern.data());
    ASSERT_NE(path, nullptr);
    root_ = path;
    ASSERT_EQ(children(), 0u);
    ASSERT_EQ(::prctl(PR_GET_CHILD_SUBREAPER, &original_subreaper_), 0);
  }
  void TearDown() override {
    EXPECT_EQ(children(), 0u) << "Generator left a live or zombie child";
    int subreaper = -1;
    EXPECT_EQ(::prctl(PR_GET_CHILD_SUBREAPER, &subreaper), 0);
    EXPECT_EQ(subreaper, original_subreaper_);
    if (!root_.empty()) fs::remove_all(root_);
  }
  fs::path source(const Json& command, const Json& params = Json::object()) {
    const auto path = root_ / "source.yaml";
    write(path, dumpYaml(Json{{"schema", "xgc2.scene-source.v1"}, {"mode", "random"}, {"format", "geometry"},
                             {"generator", {{"command", command}, {"parameters", params}}}}));
    return path;
  }
  fs::path resultSource(const std::string& payload = kResult) {
    return source(Json::array({kPython, "-c", "import os,sys;os.write(1,sys.argv[1].encode())", payload}));
  }
  void error(const std::function<void()>& call, const std::string& expected) {
    try { call(); FAIL() << "Expected SceneError: " << expected; }
    catch (const SceneError& failure) { EXPECT_EQ(failure.what(), expected); }
    EXPECT_EQ(children(), 0u);
  }
  void gone(pid_t pid) {
    errno = 0;
    EXPECT_EQ(::kill(pid, 0), -1) << "Process " << pid << " survived generator cleanup";
    EXPECT_EQ(errno, ESRCH);
  }
};

std::vector<fs::path> scenarios() {
  std::vector<fs::path> result;
  for (const auto* category : {"generation", "prepare"}) {
    for (const auto& file : fs::directory_iterator(fs::path(XGC_SCENE_GOLDEN_DIR) / "corpus" / category)) {
      if (file.path().extension() != ".json") continue;
      const auto scenario = parseJson(read(file.path()));
      if (std::any_of(scenario.at("steps").begin(), scenario.at("steps").end(), [](const Json& step) {
            return step.at("operation") == "generator.resolve";
          })) result.push_back(file.path());
    }
  }
  std::sort(result.begin(), result.end());
  return result;
}

std::string substitute(std::string value, const std::vector<std::pair<std::string, std::string>>& bindings) {
  for (const auto& binding : bindings) {
    std::size_t position = 0;
    while ((position = value.find(binding.first, position)) != std::string::npos) {
      value.replace(position, binding.first.size(), binding.second);
      position += binding.second.size();
    }
  }
  return value;
}

Json bindTokens(const Json& value, const std::vector<std::pair<std::string, std::string>>& bindings) {
  if (value.is_string()) return substitute(value.get<std::string>(), bindings);
  if (value.is_array()) {
    Json result = Json::array();
    for (const auto& child : value) result.push_back(bindTokens(child, bindings));
    return result;
  }
  if (value.is_object()) {
    Json result = Json::object();
    for (auto it = value.begin(); it != value.end(); ++it) result[substitute(it.key(), bindings)] = bindTokens(it.value(), bindings);
    return result;
  }
  return value;
}

TEST_F(GenerationTest, GoldenInventory) {
  const auto files = scenarios();
  EXPECT_EQ(files.size(), 3u);
  for (const auto& path : files) EXPECT_EQ(path.parent_path().filename(), "generation");
}

class GenerationReplay : public GenerationTest, public ::testing::WithParamInterface<fs::path> {};

TEST_P(GenerationReplay, ResolveAndDiskEffects) {
  auto scenario = parseJson(read(GetParam()));
  SCOPED_TRACE(scenario.at("scenario").get<std::string>());
  const auto work = root_ / "work", fixtures = root_ / "fixtures";
  fs::create_directories(work);
  fs::create_directories(fixtures);
  std::vector<std::pair<std::string, std::string>> bindings{{"$fixtures", fixtures.string()}, {"$work", work.string()}};
  scenario = bindTokens(scenario, bindings);
  const auto& setup = scenario.at("setup");
  for (const auto& directory : setup.at("directories")) fs::create_directories(directory.get<std::string>());
  for (const auto* field : {"files", "executables"}) {
    for (auto it = setup.at(field).begin(); it != setup.at(field).end(); ++it) {
      write(it.key(), it.value().at("contents").get<std::string>(), it.value().at("mode").get<unsigned>());
      if (std::string(field) == "executables") {
        EXPECT_EQ(read(fs::path(XGC_SCENE_GOLDEN_DIR) / "fixtures" / fs::path(it.key()).filename()), read(it.key()));
      }
    }
  }
  // Hash after substituting the executable and counter paths in source YAML.
  const auto source_path = scenario.at("steps")[0].at("request").at("source").get<std::string>();
  bindings = {{"$sourceSha256", digest(read(source_path))}};
  scenario = bindTokens(scenario, bindings);
  for (const auto& step : scenario.at("steps")) {
    const auto& request = step.at("request");
    const auto& response = step.at("response");
    ASSERT_EQ(step.at("operation"), "generator.resolve");
    auto call = [&] {
      const auto result = resolve(request.at("source").get<std::string>(), request.at("working_file").get<std::string>(),
                                  request.at("overrides"), request.value("max_bytes", kMaxDocumentBytes));
      return Json::array({Json::array({result.first.first, result.first.second}), result.second});
    };
    if (response.contains("error")) {
      ASSERT_EQ(response.at("error").at("type"), "SceneError");
      error([&] { call(); }, response.at("error").at("message").get<std::string>());
    } else EXPECT_EQ(call(), response.at("value"));
    EXPECT_EQ(children(), 0u);
    const auto& effects = step.at("effects");
    EXPECT_TRUE(effects.at("apply").empty());
    EXPECT_TRUE(effects.at("motion").empty());
    std::set<fs::path> expected_files, actual_files;
    for (auto it = effects.at("files").begin(); it != effects.at("files").end(); ++it) {
      SCOPED_TRACE(it.key());
      if (it.value().at("contents").is_null()) EXPECT_FALSE(fs::exists(it.key()));
      else {
        expected_files.insert(it.key());
        ASSERT_TRUE(fs::is_regular_file(it.key()));
        EXPECT_EQ(read(it.key()), it.value().at("contents"));
        EXPECT_EQ(static_cast<unsigned>(fs::status(it.key()).permissions()) & 0777u, it.value().at("mode"));
      }
    }
    for (const auto& file : fs::recursive_directory_iterator(work))
      if (!file.is_directory()) actual_files.insert(file.path());
    EXPECT_EQ(actual_files, expected_files) << "Unexpected artifact or temporary file";
  }
}

INSTANTIATE_TEST_SUITE_P(Corpus, GenerationReplay, ::testing::ValuesIn(scenarios()), [](const auto& info) {
  auto name = info.param.parent_path().filename().string() + "_" + info.param.stem().string();
  std::replace(name.begin(), name.end(), '-', '_');
  return name;
});

TEST_F(GenerationTest, FixedAndPlainDocumentsKeepTheirSourceDigest) {
  const Json scene{{"schema", kSchema}, {"id", "fixed"}, {"frame", "world"}, {"obstacles", Json::array()}};
  const auto path = root_ / "source.yaml";
  const auto fixed = dumpYaml(Json{{"schema", "xgc2.scene-source.v1"}, {"mode", "fixed"},
                                   {"format", "geometry"}, {"document", scene}});
  write(path, fixed);
  auto resolved = resolve(path, root_ / "ignored.yaml", {{"undeclared", 1}}, 0);
  EXPECT_EQ(resolved.first.first, document(scene));
  EXPECT_EQ(resolved.first.second, digest(fixed));
  EXPECT_EQ(resolved.second, path.string());
  write(path, dumpYaml(scene));
  resolved = resolve(path);
  EXPECT_EQ(resolved.first, load(path));
  EXPECT_EQ(resolved.second, path.string());
  EXPECT_FALSE(fs::exists(root_ / "ignored.yaml"));
}

TEST_F(GenerationTest, ExistingRunFilePrecedesGeneratorValidation) {
  const auto path = source(Json::array());
  const auto target = root_ / "scene.yaml";
  const auto scene = document(parseJson(kResult).at("scene"));
  write(target, dumpYaml(scene), 0640);
  const auto resolved = resolve(path, target, {{"unknown", true}}, 0);
  EXPECT_EQ(resolved.first, load(target));
  EXPECT_EQ(resolved.second, target.string());
  EXPECT_EQ(static_cast<unsigned>(fs::status(target).permissions()) & 0777u, 0640u);
  EXPECT_FALSE(fs::exists(root_ / "scene.generation.yaml"));
}

TEST_F(GenerationTest, SourceAndParameterValidation) {
  const auto target = root_ / "scene.yaml";
  const auto path = resultSource();
  error([&] { resolve(path); }, "Random scene requires a run-owned result file");
  const auto original = parseYaml(read(path));
  for (const auto& update : {Json{{"format", "native"}}, Json{{"mode", "other"}}, Json{{"mode", nullptr}}}) {
    auto changed = original;
    changed.update(update);
    write(path, dumpYaml(changed));
    error([&] { resolve(path, target); }, "Scene source requires explicit geometry format and fixed/random mode");
  }
  for (const auto& command : {Json(nullptr), Json("command"), Json::array(), Json::array({1}), Json::array({std::string("bad\0arg", 7)})}) {
    source(command);
    error([&] { resolve(path, target); }, "Scene generator requires an explicit argument vector");
  }
  resultSource();
  for (const auto& overrides : {Json{{"unknown", 1}}, Json::array({1}), Json("bad"), Json(true)})
    error([&] { resolve(path, target, overrides); }, "Scene generation parameters are not declared by this generator");
  for (const auto* key : {"", "-count", "some_count", "a.b", "a b", "e\xcc\x81"}) {
    source(Json::array({kPython, "-c", "raise SystemExit(99)"}), Json{{key, 1}});
    error([&] { resolve(path, target); }, "Invalid generator parameter name");
  }
  for (const auto& value : {Json(true), Json(nullptr), Json("1"), Json::array({1}),
                          Json(std::numeric_limits<double>::infinity()), Json(std::numeric_limits<double>::quiet_NaN())}) {
    source(Json::array({kPython, "-c", "raise SystemExit(99)"}), Json{{"count", value}});
    error([&] { resolve(path, target); }, "Generator parameter must be a finite number");
  }
  EXPECT_FALSE(fs::exists(target));
}

TEST_F(GenerationTest, ParametersUsePythonNumbersAndKeepDeclarationOrder) {
  const std::string program = "import json,sys;result=json.loads(sys.argv[1]);result['argv']=sys.argv[2:];print(json.dumps(result))";
  const Json params{{"count", 60}, {"negative", -0.0}, {"small", 1e-5}, {"large", 1e16},
                    {"fixed", 1e15}, {"integer", std::uint64_t{9007199254740993}}, {"caf\xc3\xa9-\xc2\xb2", 2.5}};
  const auto path = source(Json::array({kPython, "-c", program, kResult}), params);
  const auto target = root_ / "run" / "scene.yaml";
  resolve(path, target, Json{{"small", 1e-6}, {"count", 4}});
  const auto metadata = parseYaml(read(root_ / "run" / "scene.generation.yaml"));
  EXPECT_EQ(metadata.at("argv"), Json::array({"--count", "4", "--negative", "-0.0", "--small", "1e-06",
                                             "--large", "1e+16", "--fixed", "1000000000000000.0",
                                             "--integer", "9007199254740993", "--caf\xc3\xa9-\xc2\xb2", "2.5"}));
  auto expected = params;
  expected["count"] = 4;
  expected["small"] = 1e-6;
  EXPECT_EQ(metadata.at("parameters"), expected);
  EXPECT_EQ(metadata.at("sourceSha256"), digest(read(path)));
  EXPECT_EQ(metadata.at("mode"), "random");
  EXPECT_EQ(metadata.at("format"), "geometry");
}

TEST_F(GenerationTest, GeneratorResultErrorsMatchPython) {
  const std::vector<std::pair<std::string, std::string>> cases{
      {"", "Expecting value: line 1 column 1 (char 0)"},
      {"garbage", "Expecting value: line 1 column 1 (char 0)"},
      {"\xef\xbb\xbf{", "Expecting property name enclosed in double quotes: line 1 column 2 (char 1)"},
      {"{\"caf\xc3\xa9\":1,}", "Expecting property name enclosed in double quotes: line 1 column 11 (char 10)"},
      {"{", "Expecting property name enclosed in double quotes: line 1 column 2 (char 1)"},
      {"{\"a\" 1}", "Expecting ':' delimiter: line 1 column 6 (char 5)"},
      {"{\"a\":1,}", "Expecting property name enclosed in double quotes: line 1 column 8 (char 7)"},
      {"[1,]", "Expecting value: line 1 column 4 (char 3)"},
      {"{} x", "Extra data: line 1 column 4 (char 3)"},
      {"\"unterminated", "Unterminated string starting at: line 1 column 1 (char 0)"},
      {"\"\\q\"", "Invalid \\escape: line 1 column 2 (char 1)"},
      {"\"\\u12xz\"", "Invalid \\uXXXX escape: line 1 column 3 (char 2)"},
      {"\"a\nb\"", "Invalid control character at: line 1 column 3 (char 2)"},
      {"{}", "'scene'"}, {"[]", "list indices must be integers or slices, not str"},
      {"null", "'NoneType' object is not subscriptable"}, {"false", "'bool' object is not subscriptable"},
      {"1", "'int' object is not subscriptable"}, {"1.0", "'float' object is not subscriptable"},
      {"\"scene\"", "string indices must be integers"},
      {"{\"scene\":null}", "Expected an object"},
      {"{\"scene\":{\"schema\":\"bad\"}}", "Invalid fields: unknown=[], missing=['frame', 'id', 'obstacles']"},
      {"{\"scene\":{\"schema\":\"bad\",\"id\":\"bad\",\"frame\":\"world\",\"obstacles\":[]}}", "Unsupported scene schema"}};
  const auto target = root_ / "scene.yaml";
  for (const auto& entry : cases) {
    SCOPED_TRACE(entry.first);
    const auto path = resultSource(entry.first);
    error([&] { resolve(path, target); }, "Invalid generator result: " + entry.second);
    EXPECT_FALSE(fs::exists(target));
    EXPECT_FALSE(fs::exists(root_ / "scene.generation.yaml"));
  }
}

TEST_F(GenerationTest, DuplicateResultFieldsAndProvenanceFollowJsonLoads) {
  const auto path = resultSource(R"({"tag":"first","tag":"last","parameters":"replace","mode":"fixed","scene":null,"scene":{"schema":"xgc2.scene.v1","id":"generated","frame":"world","obstacles":[]},"special":NaN})");
  resolve(path, root_ / "scene.yaml");
  const auto metadata = read(root_ / "scene.generation.yaml");
  EXPECT_EQ(metadata, "tag: last\nparameters: {}\nmode: random\nspecial: .nan\nsourceSha256: " + digest(read(path)) + "\nformat: geometry\n");
  EXPECT_EQ(load(root_ / "scene.yaml").first.at("id"), "generated");
}

TEST_F(GenerationTest, ArtifactLimitAndRenameFailureLeaveNoTemporaryFiles) {
  const auto path = resultSource();
  const auto target = root_ / "run" / "scene.yaml";
  error([&] { resolve(path, target, nullptr, std::strlen(kResult)); }, "Generated scene artifacts exceed the granted byte limit");
  EXPECT_TRUE(fs::is_empty(target.parent_path()));
  fs::create_directory(target);
  EXPECT_THROW(resolve(path, target), std::system_error);
  EXPECT_TRUE(fs::is_directory(target));
  EXPECT_TRUE(fs::is_regular_file(target.parent_path() / "scene.generation.yaml"));
  std::size_t files = 0;
  for (const auto& file : fs::directory_iterator(target.parent_path())) {
    EXPECT_TRUE(file.path().filename() == "scene.yaml" || file.path().filename() == "scene.generation.yaml");
    ++files;
  }
  EXPECT_EQ(files, 2u);
}

TEST_F(GenerationTest, OversizeSourceAndFailedSpawn) {
  const auto path = root_ / "source.yaml";
  write(path, std::string(kMaxDocumentBytes + 1, 'x'));
  error([&] { resolve(path, root_ / "scene.yaml"); }, "Scene source exceeds the supported size");
  const auto missing = (root_ / "missing-generator").string();
  source(Json::array({missing}));
  error([&] { resolve(path, root_ / "scene.yaml"); }, "Scene generator failed: [Errno 2] No such file or directory: '" + missing + "'");
}

TEST_F(GenerationTest, StreamLimitsAndExitStatus) {
  EXPECT_EQ(run_generator({kPython, "-c", "import os;os.write(1,b'x'*65536);os.write(2,b'e'*65536)"}, 65536), std::string(65536, 'x'));
  EXPECT_EQ(run_generator({kPython, "-c", "pass"}, 0), "");
  error([&] { run_generator({kPython, "-c", "import os;os.write(1,b'x'*65537)"}, 65536); }, "Scene generator stdout exceeds its byte limit");
  error([&] { run_generator({kPython, "-c", "import os;os.write(2,b'e'*65537)"}, 65536); }, "Scene generator stderr exceeds its byte limit");
  error([&] { run_generator({kPython, "-c", "raise SystemExit(3)"}, 100); }, "Scene generator exited with status 3");
  error([&] { run_generator({kPython, "-c", "import os,signal;os.kill(os.getpid(),signal.SIGTERM)"}, 100); }, "Scene generator exited with status -15");
}

TEST_F(GenerationTest, BackgroundDescendantsAreKilledAndReapedOnSuccessAndFailure) {
  const auto pid_file = root_ / "pids.json";
  const std::string program =
      "import os,time,json,sys\nchild=os.fork()\n"
      "if child == 0:\n os.close(1);os.close(2);time.sleep(60)\n"
      "else:\n open(sys.argv[1],'w').write(json.dumps([os.getpid(),child]))\n print('ok')\n raise SystemExit(int(sys.argv[2]))\n";
  EXPECT_EQ(run_generator({kPython, "-c", program, pid_file.string(), "0"}, 100), "ok\n");
  for (const auto& pid : parseJson(read(pid_file))) gone(pid.get<pid_t>());
  error([&] { run_generator({kPython, "-c", program, pid_file.string(), "3"}, 100); }, "Scene generator exited with status 3");
  for (const auto& pid : parseJson(read(pid_file))) gone(pid.get<pid_t>());
}

TEST_F(GenerationTest, ConcurrentGeneratorsKeepSubreapingUntilBothGroupsFinish) {
  const std::string program =
      "import os,time,sys\nchild=os.fork()\n"
      "if child == 0:\n os.close(1);os.close(2);time.sleep(60)\n"
      "else:\n time.sleep(float(sys.argv[1]));print('ok')\n";
  auto first = std::async(std::launch::async, [&] { return run_generator({kPython, "-c", program, "0.1"}, 100); });
  auto second = std::async(std::launch::async, [&] { return run_generator({kPython, "-c", program, "0.3"}, 100); });
  EXPECT_EQ(first.get(), "ok\n");
  EXPECT_EQ(second.get(), "ok\n");
}

TEST_F(GenerationTest, ChildRestoresIgnoredPipeSignal) {
  struct RestoreSignal {
    struct sigaction previous;
    ~RestoreSignal() { ::sigaction(SIGPIPE, &previous, nullptr); }
  } restore;
  ASSERT_EQ(::sigaction(SIGPIPE, nullptr, &restore.previous), 0);
  struct sigaction ignored{};
  ignored.sa_handler = SIG_IGN;
  ::sigemptyset(&ignored.sa_mask);
  ASSERT_EQ(::sigaction(SIGPIPE, &ignored, nullptr), 0);
  error([&] { run_generator({"/bin/sh", "-c", "kill -PIPE $$; exit 7"}, 100); }, "Scene generator exited with status -13");
}

TEST_F(GenerationTest, BackgroundChildHoldingPipesIsKilledAfterBudget) {
  const auto pid_file = root_ / "pids.json";
  const std::string program =
      "import os,time,json,sys\nchild=os.fork()\n"
      "if child == 0:\n time.sleep(60)\n"
      "else:\n open(sys.argv[1],'w').write(json.dumps([os.getpid(),child]))\n print('parent finished')\n";
  const auto start = std::chrono::steady_clock::now();
  error([&] { run_generator({kPython, "-c", program, pid_file.string()}, 100); }, "Scene generator exceeded its 30 second budget");
  const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
  EXPECT_GE(elapsed, 30.0);
  EXPECT_LT(elapsed, 45.0);
  for (const auto& pid : parseJson(read(pid_file))) gone(pid.get<pid_t>());
}

}  // namespace
}  // namespace xgc_scene
