#include "xgc_scene/assembly.hpp"
#include "xgc_scene/document.hpp"
#include "xgc_scene/json_io.hpp"
#include "xgc_scene/yaml_io.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

namespace xgc_scene {
namespace {

namespace fs = std::filesystem;

std::vector<fs::path> scenarios() {
  std::vector<fs::path> result;
  for (const auto& file : fs::directory_iterator(fs::path(XGC_SCENE_GOLDEN_DIR) / "corpus/assembly"))
    if (file.path().extension() == ".json") result.push_back(file.path());
  std::sort(result.begin(), result.end());
  return result;
}

Json readScenario(const fs::path& path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) throw std::runtime_error("Cannot read corpus file " + path.string());
  return Json::parse(stream);
}

using File = std::unique_ptr<FILE, decltype(&std::fclose)>;

File temporaryFile() {
  File result(std::tmpfile(), &std::fclose);
  if (!result) throw std::runtime_error("Cannot open CLI test stream");
  return result;
}

std::string readStream(FILE* stream) {
  std::rewind(stream);
  std::string result;
  char buffer[8192];
  while (const auto count = std::fread(buffer, 1, sizeof(buffer), stream)) result.append(buffer, count);
  if (std::ferror(stream)) throw std::runtime_error("Cannot read CLI test stream");
  return result;
}

Json runCli(const std::string& bytes) {
  const auto executable = fs::read_symlink("/proc/self/exe").parent_path() / "xgc2-scene-assemble";
  if (!fs::is_regular_file(executable)) throw std::runtime_error("Missing assembly CLI " + executable.string());
  auto input = temporaryFile(), output = temporaryFile(), error = temporaryFile();
  if (std::fwrite(bytes.data(), 1, bytes.size(), input.get()) != bytes.size())
    throw std::runtime_error("Cannot write CLI test input");
  std::rewind(input.get());
  const auto child = ::fork();
  if (child < 0) throw std::runtime_error("Cannot fork assembly CLI");
  if (child == 0) {
    if (::dup2(::fileno(input.get()), STDIN_FILENO) < 0 || ::dup2(::fileno(output.get()), STDOUT_FILENO) < 0 ||
        ::dup2(::fileno(error.get()), STDERR_FILENO) < 0) ::_exit(126);
    ::execl(executable.c_str(), executable.c_str(), static_cast<char*>(nullptr));
    ::_exit(127);
  }
  int status = 0;
  pid_t waited;
  do { waited = ::waitpid(child, &status, 0); } while (waited < 0 && errno == EINTR);
  if (waited < 0) {
    ::kill(child, SIGKILL);
    while (::waitpid(child, &status, 0) < 0 && errno == EINTR) {}
    throw std::runtime_error("Cannot wait for assembly CLI");
  }
  return Json{{"exit_code", WIFEXITED(status) ? WEXITSTATUS(status) : -WTERMSIG(status)},
              {"stdout", readStream(output.get())}, {"stderr", readStream(error.get())}};
}

Json runLibrary(const std::string& bytes) {
  try {
    const auto result = assembleJson(bytes);
    return Json{{"exit_code", 0}, {"stdout", result.dump(-1, ' ', true)}, {"stderr", ""}};
  } catch (const SceneError& error) {
    return Json{{"exit_code", 2}, {"stdout", ""}, {"stderr", std::string(error.what()) + '\n'}};
  }
}

void sameOutput(const Json& actual, const Json& expected) {
  EXPECT_EQ(actual.at("exit_code"), expected.at("exit_code"));
  EXPECT_EQ(actual.at("stderr"), expected.at("stderr"));
  const auto& stdout_actual = actual.at("stdout").get_ref<const std::string&>();
  const auto& stdout_expected = expected.at("stdout").get_ref<const std::string&>();
  // Avoid printing multi-megabyte output if a boundary assertion fails.
  EXPECT_TRUE(stdout_actual == stdout_expected) << "CLI stdout differs; actual bytes=" << stdout_actual.size()
                                              << ", expected bytes=" << stdout_expected.size();
  if (expected.at("exit_code") == 0 && actual.at("exit_code") == 0) {
    const auto value = parseJson(stdout_actual), recorded = parseJson(stdout_expected);
    EXPECT_EQ(value.at("task"), recorded.at("task"));
    const auto& yaml = value.at("sceneDocument").get_ref<const std::string&>();
    EXPECT_TRUE(yaml == recorded.at("sceneDocument").get_ref<const std::string&>());
    EXPECT_NO_THROW(document(parseYaml(yaml)));
  }
}

class AssemblyReplay : public ::testing::TestWithParam<fs::path> {};

TEST_P(AssemblyReplay, Library) {
  const auto scenario = readScenario(GetParam());
  std::size_t index = 0;
  for (const auto& step : scenario.at("steps")) {
    SCOPED_TRACE(index++);
    ASSERT_EQ(step.at("operation"), "assembly.cli");
    sameOutput(runLibrary(step.at("request").at("stdin").get<std::string>()), step.at("response").at("value"));
  }
}

TEST_P(AssemblyReplay, Cli) {
  const auto scenario = readScenario(GetParam());
  std::size_t index = 0;
  for (const auto& step : scenario.at("steps")) {
    SCOPED_TRACE(index++);
    ASSERT_EQ(step.at("operation"), "assembly.cli");
    sameOutput(runCli(step.at("request").at("stdin").get<std::string>()), step.at("response").at("value"));
  }
}

INSTANTIATE_TEST_SUITE_P(Golden, AssemblyReplay, ::testing::ValuesIn(scenarios()),
                        [](const ::testing::TestParamInfo<fs::path>& info) {
                          auto name = info.param.stem().string();
                          std::replace(name.begin(), name.end(), '-', '_');
                          return name;
                        });

Json emptyInput() {
  const auto source = dumpYaml(Json{{"schema", kSchema}, {"id", "empty"}, {"frame", "world"},
                                    {"obstacles", Json::array()}});
  return Json{{"request", Json{{"fidelity", "geometry-only"}, {"task", "task"}, {"name", "Empty World"},
                                {"elements", Json::array({Json{{"id", "a"}}, Json{{"id", "b"}}})}}},
              {"documents", Json::array({source, source})}, {"extrinsic", "parent_frame: world\n"}};
}

void rejects(const std::function<void()>& call, const char* message) {
  try { call(); FAIL() << "Expected " << message; }
  catch (const SceneError& error) { EXPECT_STREQ(error.what(), message); }
}

TEST(Assembly, InputByteBoundary) {
  auto bytes = emptyInput().dump();
  const auto expected = runLibrary(bytes);
  ASSERT_EQ(expected.at("exit_code"), 0);
  bytes.resize(4 * kMaxDocumentBytes, ' ');
  sameOutput(runLibrary(bytes), expected);
  sameOutput(runCli(bytes), expected);
  bytes.push_back(' ');
  const Json too_large{{"exit_code", 2}, {"stdout", ""}, {"stderr", "Assembly input exceeds the supported size\n"}};
  sameOutput(runLibrary(bytes), too_large);
  sameOutput(runCli(bytes), too_large);
}

Json prism() {
  constexpr int sides = 12;
  Json vertices = Json::array(), triangles = Json::array();
  const auto triangle = [&](int a, int b, int c) {
    triangles.push_back(a); triangles.push_back(b); triangles.push_back(c);
  };
  for (int z = 0; z < 2; ++z) {
    for (int i = 0; i < sides; ++i) {
      const double angle = 2 * std::acos(-1.0) * i / sides;
      vertices.push_back(Json::array({std::cos(angle), std::sin(angle), static_cast<double>(z)}));
    }
  }
  for (int i = 1; i < sides - 1; ++i) {
    triangle(0, i + 1, i);
    triangle(sides, sides + i, sides + i + 1);
  }
  for (int i = 0; i < sides; ++i) {
    const int next = (i + 1) % sides;
    triangle(i, next, sides + next);
    triangle(i, sides + next, sides + i);
  }
  return geometry(Json{{"type", "convex"}, {"vertices", vertices}, {"triangles", triangles}});
}

Json outputBoundaryInput() {
  constexpr std::size_t obstacle_count = 64;
  const auto part = Json{{"id", "p000"}, {"geometry", prism()}};
  auto single = obstacle(Json{{"id", "o000"}, {"name", "N"}, {"parts", Json::array({part})}});
  auto twice = single;
  twice["parts"].push_back(twice.at("parts")[0]);
  twice["parts"][1]["id"] = "p001";
  const auto part_bytes = dumpYaml(Json::array({twice})).size() - dumpYaml(Json::array({single})).size();
  auto candidate = Json{{"schema", kSchema}, {"id", "boundary"}, {"frame", "world"}, {"obstacles", Json::array()}};
  for (std::size_t i = 0; i < obstacle_count; ++i) {
    auto placed = single;
    const auto eid = i < obstacle_count / 2 ? "a" : "b";
    placed["id"] = std::string(eid) + ".o" + std::to_string(1000 + i);
    placed["name"] = std::string(eid) + " N";
    candidate["obstacles"].push_back(std::move(placed));
  }
  const auto base_bytes = dumpYaml(candidate).size();
  if (base_bytes >= kMaxDocumentBytes) throw std::runtime_error("Boundary fixture base is too large");
  auto extra_parts = (kMaxDocumentBytes - base_bytes) / part_bytes;
  if (extra_parts + obstacle_count > 4096) throw std::runtime_error("Boundary fixture exceeds part count");
  for (auto& item : candidate["obstacles"]) {
    while (extra_parts && item["parts"].size() < 128) {
      auto added = single.at("parts")[0];
      added["id"] = "p" + std::to_string(1000 + item["parts"].size()).substr(1);
      item["parts"].push_back(std::move(added));
      --extra_parts;
    }
  }
  if (extra_parts) throw std::runtime_error("Boundary fixture exceeds per-obstacle part count");
  auto padding = kMaxDocumentBytes - dumpYaml(candidate).size();
  for (auto& item : candidate["obstacles"]) {
    // Keep each name on one YAML line, so each added ASCII character is one byte.
    const auto added = std::min<std::size_t>(padding, 64);
    item["name"] = item.at("name").get<std::string>() + std::string(added, 'N');
    padding -= added;
  }
  if (padding || dumpYaml(candidate).size() != kMaxDocumentBytes)
    throw std::runtime_error("Cannot construct exact output boundary");
  auto input = emptyInput();
  input["request"]["name"] = "boundary";
  auto a = Json{{"schema", kSchema}, {"id", "source"}, {"frame", "world"}, {"obstacles", Json::array()}};
  auto b = a;
  for (auto item : candidate.at("obstacles")) {
    const bool first = item.at("id").get<std::string>()[0] == 'a';
    item["id"] = item.at("id").get<std::string>().substr(2);
    item["name"] = item.at("name").get<std::string>().substr(2);
    (first ? a : b)["obstacles"].push_back(std::move(item));
  }
  input["documents"] = Json::array({dumpYaml(a), dumpYaml(b)});
  return input;
}

TEST(Assembly, OutputByteBoundary) {
  auto input = outputBoundaryInput();
  const auto result = assemble(input.at("request"), input.at("documents"), input.at("extrinsic"));
  ASSERT_EQ(result.at("sceneDocument").get_ref<const std::string&>().size(), kMaxDocumentBytes);
  const Json expected{{"exit_code", 0}, {"stdout", result.dump(-1, ' ', true)}, {"stderr", ""}};
  sameOutput(runCli(input.dump()), expected);
  auto source = parseYaml(input.at("documents")[1].get<std::string>());
  source["obstacles"].back()["name"] = source.at("obstacles").back().at("name").get<std::string>() + "N";
  input["documents"][1] = dumpYaml(source);
  rejects([&] { assemble(input.at("request"), input.at("documents"), input.at("extrinsic")); },
          "Assembled scene exceeds the supported size");
  sameOutput(runCli(input.dump()), Json{{"exit_code", 2}, {"stdout", ""},
                                      {"stderr", "Assembled scene exceeds the supported size\n"}});
}

TEST(Assembly, ElementAndTaskBoundaries) {
  auto input = emptyInput();
  input["request"]["task"] = " \t" + std::string(64, 't') + "\n";
  input["request"]["name"] = "  Mixed_Name World  ";
  const auto source = input.at("documents")[0];
  for (int i = 2; i < 64; ++i) {
    input["request"]["elements"].push_back(Json{{"id", "e" + std::to_string(i)}});
    input["documents"].push_back(source);
  }
  const auto result = assemble(input.at("request"), input.at("documents"), input.at("extrinsic"));
  EXPECT_EQ(result.at("task"), std::string(64, 't'));
  EXPECT_EQ(parseYaml(result.at("sceneDocument").get<std::string>()).at("id"), "mixed-name-world");
  input["request"]["name"] = "\xe2\x84\xaa" "ettle";
  const auto lowercase = assemble(input.at("request"), input.at("documents"), input.at("extrinsic"));
  EXPECT_EQ(parseYaml(lowercase.at("sceneDocument").get<std::string>()).at("id"), "kettle");
  input["request"]["elements"].push_back(Json{{"id", "e64"}});
  input["documents"].push_back(source);
  rejects([&] { assemble(input.at("request"), input.at("documents"), input.at("extrinsic")); },
          "An assembled world needs 2 to 64 elements");
}

TEST(Assembly, CliCaughtErrorsAndJsonDiagnostics) {
  const std::vector<std::pair<std::string, std::string>> errors{
      {"", "Expecting value: line 1 column 1 (char 0)"},
      {"\xef\xbb\xbf{", "Expecting property name enclosed in double quotes: line 1 column 2 (char 1)"},
      {"{\"\xc3\xa9\" 0}", "Expecting ':' delimiter: line 1 column 6 (char 5)"},
      {"{\"request\" 0}", "Expecting ':' delimiter: line 1 column 12 (char 11)"},
      {"{\"request\":0,}", "Expecting property name enclosed in double quotes: line 1 column 14 (char 13)"},
      {"[0 1]", "Expecting ',' delimiter: line 1 column 4 (char 3)"},
      {"{} []", "Extra data: line 1 column 4 (char 3)"},
      {"\"abc", "Unterminated string starting at: line 1 column 1 (char 0)"},
      {"\"\\q\"", "Invalid \\escape: line 1 column 2 (char 1)"},
      {"\"\\uQQQQ\"", "Invalid \\uXXXX escape: line 1 column 3 (char 2)"},
      {"[\"a\n\"]", "Invalid control character at: line 1 column 4 (char 3)"},
      {"{\"request\":0,\"request\":NaN}", "Nonfinite JSON"},
      {"{\"request\":0,\"request\":{\"a\":0,\"a\":1}}", "Duplicate scene field a"},
      {"[]", "list indices must be integers or slices, not str"},
      {"{\"request\":null,\"documents\":[],\"extrinsic\":\"\"}", "'NoneType' object has no attribute 'get'"}};
  for (const auto& sample : errors) {
    SCOPED_TRACE(sample.first);
    const Json expected{{"exit_code", 2}, {"stdout", ""}, {"stderr", sample.second + '\n'}};
    sameOutput(runLibrary(sample.first), expected);
    sameOutput(runCli(sample.first), expected);
  }
  auto input = emptyInput();
  const auto valid = runLibrary(input.dump());
  sameOutput(runLibrary("\xef\xbb\xbf" + input.dump()), valid);
  sameOutput(runCli("\xef\xbb\xbf" + input.dump()), valid);
  input["request"]["task"] = nullptr;
  sameOutput(runCli(input.dump()), Json{{"exit_code", 2}, {"stdout", ""},
                                      {"stderr", "'NoneType' object has no attribute 'strip'\n"}});
  input = emptyInput();
  input["documents"][0] = 1;
  sameOutput(runCli(input.dump()), Json{{"exit_code", 2}, {"stdout", ""},
                                      {"stderr", "'int' object has no attribute 'read'\n"}});
  input = emptyInput();
  input["documents"][0] = "obstacles: [\n";
  const auto actual = runCli(input.dump());
  EXPECT_EQ(actual.at("exit_code"), 2);
  EXPECT_EQ(actual.at("stdout"), "");
  EXPECT_EQ(actual.at("stderr").get<std::string>().substr(0, 20), "Invalid scene YAML: ");
}

}  // namespace
}  // namespace xgc_scene
