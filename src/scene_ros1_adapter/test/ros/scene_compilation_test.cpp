#include "xgc_scene/document.hpp"
#include "xgc_scene/json_io.hpp"
#include "xgc_scene/node.hpp"
#include "xgc_scene/prepare.hpp"
#include "xgc_scene/yaml_io.hpp"

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <unistd.h>

namespace xgc_scene {
namespace {
namespace fs = std::filesystem;

class CompilationOutputs final : public RosOutputs {
 public:
  void snapshot(const xgc2_geometry_msgs::SceneSnapshot& value) override { snapshots.push_back(value); }
  void state(const xgc2_geometry_msgs::SceneState&) override {}
  void document(const std::string& value) override { documents.push_back(parseJson(value)); }
  void markers(const visualization_msgs::MarkerArray&) override {}
  void staticTransforms(const std::vector<geometry_msgs::TransformStamped>&) override {}
  void dynamicTransforms(const std::vector<geometry_msgs::TransformStamped>&) override {}
  std::vector<xgc2_geometry_msgs::SceneSnapshot> snapshots;
  std::vector<Json> documents;
};

// Only the external application transport is fake. Resource loading and the
// compiled definition are produced by the real SceneNode/compile path.
class ImmediateSimulation final : public Simulation {
 public:
  void update(const Json& compiled, const std::string& epoch, std::uint64_t revision,
              const Json&) override {
    definitions.push_back(Json{{"compiled", compiled}, {"epoch", epoch}, {"revision", revision}});
    report = Json{{"epoch", epoch}, {"revision", revision}, {"applied", true},
                  {"operational", true}, {"capability", "simulation"}};
  }
  std::optional<Json> takeReport() override {
    auto value = std::move(report);
    report.reset();
    return value;
  }
  void close() override {}
  std::vector<Json> definitions;
  std::optional<Json> report;
};

class SceneCompilation : public testing::Test {
 protected:
  void SetUp() override {
    char pattern[] = "/tmp/xgc-scene-compilation-XXXXXX";
    const auto directory = ::mkdtemp(pattern);
    ASSERT_NE(directory, nullptr);
    root = directory;
    writeModel("first.sdf", "1 2 3");
    writeModel("second.sdf", "4 5 6");
  }
  void TearDown() override { if (!root.empty()) fs::remove_all(root); }
  void writeModel(const std::string& file, const std::string& size) {
    std::ofstream output(root / file);
    output << "<sdf version='1.7'><model name='resource'><link name='body'>"
              "<collision name='solid'><geometry><box><size>" << size <<
              "</size></box></geometry></collision></link></model></sdf>";
    ASSERT_TRUE(output.good());
  }
  Json source(const std::string& file, std::size_t count = 100) const {
    Json obstacles = Json::array();
    for (std::size_t i = 0; i < count; ++i)
      obstacles.push_back(Json{{"id", "instance" + std::to_string(i)}, {"model", "shared"},
                               {"pose", Json{{"position", Json::array({static_cast<double>(i) * 2, 0, 0})}}}});
    return Json{{"schema", "xgc2.scene.v1"}, {"id", "compiled-cache"}, {"frame", "world"},
                {"models", Json{{"shared", Json{{"resource", Json{{"format", "sdformat-model"}, {"file", file}}}}}}},
                {"obstacles", std::move(obstacles)}};
  }
  SceneParameters parameters(const Json& initial, bool native = false) const {
    std::ofstream output(root / "scene.yaml");
    output << dumpYaml(initial);
    output.close();
    SceneParameters result;
    result.profile = native ? "gazebo" : "physical";
    result.scene_file = root / "scene.yaml";
    return result;
  }
  Json request(SceneNode& node, const std::string& operation, const std::string& id,
               Json fields = Json::object()) const {
    const auto current = node.envelope();
    fields["operation"] = operation;
    fields["requestId"] = id;
    fields["expectedEpoch"] = current.at("epoch");
    fields["expectedRevision"] = current.at("revision");
    return fields;
  }
  fs::path root;
  CompilationOutputs outputs;
  double now = 100;
};

TEST_F(SceneCompilation, HundredInstancesKeepOneCompiledGeometryAfterResourceRemoval) {
  const auto initial = source("first.sdf");
  SceneNode node(initial, parameters(initial), "/compiled", outputs, [&] { return now; });
  const auto before = node.envelope();
  ASSERT_EQ(before.at("compiled").at("geometries").size(), 1u);
  ASSERT_EQ(before.at("compiled").at("obstacles").size(), 100u);
  EXPECT_EQ(before.at("document").at("models").at("shared").at("resource").at("file"), "first.sdf");
  EXPECT_FALSE(before.at("document").at("models").at("shared").contains("parts"));
  ASSERT_EQ(outputs.snapshots.back().geometries.size(), 1u);
  ASSERT_EQ(outputs.snapshots.back().obstacles.size(), 100u);
  // Any subsequent real resource compilation must fail: both the original
  // pathname and the renamed backing file are gone before the hot operations.
  fs::rename(root / "first.sdf", root / "renamed.sdf");
  fs::remove(root / "renamed.sdf");
  EXPECT_THROW(compile(before.at("document"), root.string()), SceneError);
  for (int i = 0; i < 20; ++i) {
    now += .01;
    EXPECT_NO_THROW(node.tick());
    EXPECT_EQ(node.envelope().at("compiled"), before.at("compiled"));
  }
  ASSERT_TRUE(node.commandValue(request(node, "play", "play")).at("success"));
  now += .5;
  EXPECT_NO_THROW(node.tick());
  ASSERT_TRUE(node.commandValue(request(node, "pause", "pause")).at("success"));
  EXPECT_EQ(node.envelope().at("document"), before.at("document"));
  EXPECT_EQ(node.envelope().at("compiled"), before.at("compiled"));
}

TEST_F(SceneCompilation, StartupConsumesPreparedDefinitionWithoutReopeningResources) {
  const auto initial = source("first.sdf");
  const auto asset = root / "Shared/Scenes/fixture";
  const auto workspace = root / "workspace";
  fs::create_directories(asset);
  fs::create_directory(workspace);
  fs::rename(root / "first.sdf", asset / "first.sdf");
  std::ofstream(asset / "scene.yaml") << dumpYaml(initial);
  std::ofstream(asset / "manifest.json") << dumpJson(Json{
      {"schemaVersion", 1}, {"kind", "xgc.scene-replay-asset.v1"}, {"revisionId", "fixture1"},
      {"simulators", {{"xsim", {{"geometry", "document"}}}}},
      {"sceneDocument", {{"file", "scene.yaml"}, {"sha256", sha256(asset / "scene.yaml")}}}});
  const auto selected = inputs(Json{{"sessionId", "session1"}, {"experimentResourceId", "experiment1"},
      {"scene", {{"asset", "fixture"}, {"simulator", "xsim"}, {"parameters", {{"frozen", true}}}}}}, root, workspace);
  SceneParameters config;
  config.profile = "xsim";
  config.scene_file = selected.at("sceneFile").get<std::string>();
  config.resource_root = selected.at("assetDirectory").get<std::string>();
  config.compiled_scene_file = selected.at("compiledSceneFile").get<std::string>();
  config.frozen = selected.at("frozen").get<bool>();
  const auto prepared = parseJson(read(config.compiled_scene_file, kMaxDocumentBytes));
  fs::remove(asset / "first.sdf");
  // This invokes real startup resolution, Store initialization and projection.
  // A second SDF parse would fail because its only resource is already gone.
  SceneNode node(config, "/prepared", outputs, [&] { return now; });
  const auto current = node.envelope();
  EXPECT_EQ(current.at("document"), document(initial));
  EXPECT_EQ(current.at("compiled"), prepared);
  EXPECT_EQ(current.at("revision"), 1);
  ASSERT_EQ(outputs.snapshots.back().geometries.size(), 1u);
}

TEST_F(SceneCompilation, WorkingFileDoesNotMoveTheAssetResourceRoot) {
  const auto initial = source("first.sdf", 1);
  auto config = parameters(initial);
  config.working_file = root / "workspace" / "working.yaml";
  fs::create_directory(config.working_file.parent_path());
  std::ofstream(config.working_file) << dumpYaml(initial);
  SceneNode node(config, "/working", outputs, [&] { return now; });
  // Both startup and an accepted edit resolve resources in the sealed asset,
  // never under workspace/. Edits still use the normal compile hook.
  auto changed = node.envelope().at("document");
  changed["obstacles"][0]["name"] = "edited";
  auto command = request(node, "replace", "working-edit", {{"document", changed}});
  const auto result = node.commandValue(command);
  ASSERT_TRUE(result.at("success")) << result;
  EXPECT_EQ(result.at("compiled").at("geometries").size(), 1u);
  EXPECT_EQ(result.at("document").at("obstacles")[0].at("name"), "edited");
}

TEST_F(SceneCompilation, OldRequestReplayRetainsItsSourceAndCompiledDefinitionTogether) {
  const auto initial = source("first.sdf");
  SceneNode node(initial, parameters(initial), "/compiled", outputs, [&] { return now; });
  const auto original_request = request(node, "play", "old-request");
  const auto old_receipt = node.commandValue(original_request);
  ASSERT_TRUE(old_receipt.at("success"));
  const auto update = node.commandValue(request(node, "replace", "new-definition",
                                               Json{{"document", source("second.sdf")}}));
  ASSERT_TRUE(update.at("success")) << update;
  const auto current = node.envelope();
  ASSERT_NE(current.at("compiled"), old_receipt.at("compiled"));
  ASSERT_NE(current.at("document"), old_receipt.at("document"));
  fs::remove(root / "first.sdf");
  fs::remove(root / "second.sdf");
  EXPECT_EQ(node.commandValue(original_request), old_receipt);
  const auto retained = node.requestResult("old-request");
  ASSERT_TRUE(retained);
  EXPECT_EQ(*retained, old_receipt);
  EXPECT_EQ(node.envelope().at("compiled"), current.at("compiled"));
  EXPECT_EQ(node.envelope().at("document"), current.at("document"));
  EXPECT_EQ(node.envelope().at("revision"), current.at("revision"));
}

TEST_F(SceneCompilation, InvalidResourceUpdateDoesNotReplaceTheCurrentCompiledPair) {
  const auto initial = source("first.sdf");
  SceneNode node(initial, parameters(initial), "/compiled", outputs, [&] { return now; });
  const auto before = node.envelope();
  const auto publications = outputs.documents.size();
  const auto failure = node.commandValue(request(node, "replace", "bad-definition",
                                                Json{{"document", source("missing.sdf")}}));
  EXPECT_FALSE(failure.at("success"));
  const auto after = node.envelope();
  EXPECT_EQ(after.at("document"), before.at("document"));
  EXPECT_EQ(after.at("compiled"), before.at("compiled"));
  EXPECT_EQ(after.at("epoch"), before.at("epoch"));
  EXPECT_EQ(after.at("revision"), before.at("revision"));
  for (std::size_t i = publications; i < outputs.documents.size(); ++i) {
    EXPECT_EQ(outputs.documents[i].at("document"), before.at("document"));
    EXPECT_EQ(outputs.documents[i].at("compiled"), before.at("compiled"));
    EXPECT_EQ(outputs.documents[i].at("revision"), before.at("revision"));
  }
}

TEST_F(SceneCompilation, ImmediateNativeReportsNeverPublishMixedRevisionDefinitions) {
  const auto initial = source("first.sdf", 2);
  auto simulation = std::make_shared<ImmediateSimulation>();
  SceneNode node(initial, parameters(initial, true), "/compiled", outputs,
                 [&] { return now; }, simulation);
  const auto first = node.envelope();
  const auto updated = node.commandValue(request(node, "replace", "replacement",
                                                Json{{"document", source("second.sdf", 3)}}));
  ASSERT_TRUE(updated.at("success")) << updated;
  const auto second = node.envelope();
  ASSERT_NE(first.at("revision"), second.at("revision"));
  ASSERT_NE(first.at("compiled"), second.at("compiled"));
  ASSERT_GE(outputs.documents.size(), 2u);
  for (const auto& publication : outputs.documents) {
    const auto& expected = publication.at("revision") == first.at("revision") ? first : second;
    EXPECT_EQ(publication.at("revision"), expected.at("revision"));
    EXPECT_EQ(publication.at("document"), expected.at("document"));
    EXPECT_EQ(publication.at("compiled"), expected.at("compiled"));
    EXPECT_EQ(publication.at("compiled"), compile(publication.at("document"), root.string()));
  }
  ASSERT_EQ(simulation->definitions.size(), 2u);
  for (const auto& definition : simulation->definitions) {
    const auto& expected = definition.at("revision") == first.at("revision") ? first : second;
    EXPECT_EQ(definition.at("compiled"), expected.at("compiled"));
    EXPECT_EQ(definition.at("epoch"), expected.at("epoch"));
  }
}

}  // namespace
}  // namespace xgc_scene
