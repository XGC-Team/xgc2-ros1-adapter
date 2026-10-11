#include "xgc_scene/yaml_io.hpp"
#include "xgc_scene/json_io.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <unistd.h>

namespace xgc_scene {
namespace {

struct YamlFixture { const char* name; const char* json; const char* yaml; };
#include "yaml_fixtures.hpp"

TEST(YamlIo, MatchesPyYamlScalarAndCollectionFixtures) {
  for (const auto& fixture : kYamlFixtures) {
    SCOPED_TRACE(fixture.name);
    const auto value = parseJson(fixture.json);
    EXPECT_EQ(dumpYaml(value), fixture.yaml);
    EXPECT_EQ(parseYaml(fixture.yaml), value);
  }
}

TEST(YamlIo, ResolvesYaml11ScalarsAndExplicitTags) {
  const auto value = parseYaml("[yes, NO, on, off, true, null, ~, 012, 0x10, 0b10, 1:02, 1:02.5, 1e3, 1.0e+3, !!str 42, !!float 1e3, !!int 12]\n");
  EXPECT_EQ(value, parseJson(R"([true,false,true,false,true,null,null,10,16,2,62,62.5,"1e3",1000.0,"42",1000.0,12])"));
  const auto nonfinite = parseYaml("[.nan, .inf, -.Inf]\n");
  EXPECT_TRUE(std::isnan(nonfinite[0].get<double>()));
  EXPECT_EQ(nonfinite[1].get<double>(), std::numeric_limits<double>::infinity());
  EXPECT_EQ(nonfinite[2].get<double>(), -std::numeric_limits<double>::infinity());
  EXPECT_EQ(dumpYaml(nonfinite), "- .nan\n- .inf\n- -.inf\n");
  EXPECT_EQ(parseYaml("\"a\\L  b\\P  c\"\n"), "a\xe2\x80\xa8  b\xe2\x80\xa9  c");
}

void error(const std::string& yaml, const std::string& message) {
  try { parseYaml(yaml); FAIL() << "Expected " << message; }
  catch (const SceneError& exception) { EXPECT_EQ(exception.what(), message); }
}

TEST(YamlIo, RejectsDuplicateAndNonstringKeysIncludingMerges) {
  error("a: 1\na: 2\n", "Duplicate scene field a");
  error("nested: {a: 1, a: 2}\n", "Duplicate scene field a");
  error("true: 1\n", "Scene field names must be strings");
  error("42: 1\n", "Scene field names must be strings");
  error("null: 1\n", "Scene field names must be strings");
  error("2001-12-15: 1\n", "Scene field names must be strings");
  error("? [a, b]\n: 1\n", "Scene field names must be strings");
  error("base: &base {a: 1}\nitem: {<<: *base, a: 2}\n", "Duplicate scene field a");
  error("item: {<<: [{a: 1}, {a: 2}]}\n", "Duplicate scene field a");
  EXPECT_EQ(dumpYaml(parseYaml("item: {<<: [{a: 1}, {b: 2}], c: 3}\n")), "item:\n  b: 2\n  a: 1\n  c: 3\n");
  EXPECT_EQ(dumpYaml(parseYaml("item: {<<: {a: 1}, b: 2}\n")), "item:\n  a: 1\n  b: 2\n");
  EXPECT_EQ(parseYaml("!!str true: 1\n'<<': 2\n=: 3\n"), (Json{{"true", 1}, {"<<", 2}, {"=", 3}}));
}

TEST(YamlIo, KeepsStablePrefixForParserDiagnostics) {
  for (const auto& value : {"a: [", "---\na: 1\n---\nb: 2\n", "a: !!python/object {}\n", "a: &a {b: *a}\n"}) {
    try { parseYaml(value); FAIL() << value; }
    catch (const SceneError& exception) { EXPECT_EQ(std::string(exception.what()).substr(0, 20), "Invalid scene YAML: "); }
  }
  EXPECT_EQ(parseYaml(""), nullptr);
}

TEST(YamlIo, FileDigestsAndSizeGuard) {
  char name[] = "/tmp/xgc-scene-yaml-XXXXXX";
  const int fd = mkstemp(name);
  ASSERT_NE(fd, -1);
  ASSERT_EQ(close(fd), 0);
  struct Cleanup { const char* path; ~Cleanup() { std::filesystem::remove(path); } } cleanup{name};
  const std::string bytes = "schema: xgc2.scene.v1\nid: fixture\nframe: world\nobstacles: []\n";
  { std::ofstream stream(name, std::ios::binary); stream << bytes; }
  const auto loaded = load(name);
  EXPECT_EQ(loaded.first.at("id"), "fixture");
  EXPECT_EQ(loaded.second, digest(bytes));
  EXPECT_EQ(fileDigest(name), loaded.second);
  EXPECT_EQ(digest("abc"), "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  { std::ofstream stream(name, std::ios::binary); stream << std::string(kMaxDocumentBytes + 1, ' '); }
  for (bool validate : {false, true}) {
    try { if (validate) load(name); else fileDigest(name); FAIL(); }
    catch (const SceneError& exception) { EXPECT_STREQ(exception.what(), "Scene file exceeds the supported size"); }
  }
}

}  // namespace
}  // namespace xgc_scene
