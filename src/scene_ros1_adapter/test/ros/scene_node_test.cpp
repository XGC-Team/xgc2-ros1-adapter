#include "xgc_scene/node.hpp"
#include "xgc_scene/document.hpp"
#include "xgc_scene/json_io.hpp"
#include "xgc_scene/motion.hpp"
#include "xgc_scene/yaml_io.hpp"

#include <gtest/gtest.h>
#include <ros/serialization.h>

#include <cmath>
#include <condition_variable>
#include <filesystem>
#include <fstream>
#include <future>
#include <mutex>
#include <unistd.h>

namespace xgc_scene {
namespace {
using namespace std::chrono;

class Outputs final : public RosOutputs {
 public:
  void snapshot(const xgc2_geometry_msgs::SceneSnapshot& value) override { snapshots.push_back(value); }
  void state(const xgc2_geometry_msgs::SceneState& value) override { states.push_back(value); }
  void document(const std::string& value) override { documents.push_back(value); }
  void markers(const visualization_msgs::MarkerArray& value) override { markers_list.push_back(value); }
  void staticTransforms(const std::vector<geometry_msgs::TransformStamped>& value) override { fixed.push_back(value); }
  void dynamicTransforms(const std::vector<geometry_msgs::TransformStamped>& value) override { moving.push_back(value); }
  std::vector<xgc2_geometry_msgs::SceneSnapshot> snapshots;
  std::vector<xgc2_geometry_msgs::SceneState> states;
  std::vector<std::string> documents;
  std::vector<visualization_msgs::MarkerArray> markers_list;
  std::vector<std::vector<geometry_msgs::TransformStamped>> fixed, moving;
};

class FakeSimulation final : public Simulation {
 public:
  void update(const Json& document, const std::string& epoch, std::uint64_t revision, const Json& motion) override {
    if (applies.empty() || applies.back().at("epoch") != epoch || applies.back().at("revision") != revision)
      applies.push_back(Json{{"document", document}, {"epoch", epoch}, {"revision", revision}});
    if (motion.value("operation", std::string()) != "")
      motions.push_back(Json{{"operation", motion.at("operation")}, {"epoch", epoch}, {"revision", revision}});
    pending = Json{{"epoch", epoch}, {"revision", revision}, {"applied", !failure},
                   {"operational", !failure}, {"message", failure ? "native outcome unknown" : "Native scene operation completed"}};
  }
  std::optional<Json> takeReport() override {
    if (hold) return {};
    auto value = std::move(pending); pending.reset(); return value;
  }
  void close() override { ++closes; }
  std::optional<Json> pending;
  bool hold = false;
  std::vector<Json> applies, motions;
  bool failure = false;
  int closes = 0;
};

Json obstacleAt(int index, bool moving = false) {
  Json result{{"id", "o" + std::to_string(index)}, {"name", "Obstacle " + std::to_string(index)},
              {"pose", Json{{"position", Json::array({index * 2.0, 1.0, 0.5})}}},
              {"parts", Json::array({Json{{"id", "base"}, {"geometry", Json{{"type", "box"}, {"size", Json::array({0.5, 0.5, 1.0})}}}},
                                      Json{{"id", "post"}, {"geometry", Json{{"type", "cylinder"}, {"radius", 0.1}, {"height", 1.0}}}}})}};
  if (moving) result["motion"] = Json{{"type", "constant_twist"}, {"linear", Json::array({0.1, 0.0, 0.0})},
                                      {"angular", Json::array({0.0, 0.0, 0.2})}};
  return result;
}
Json scene(int count = 5) {
  Json obstacles = Json::array();
  for (int i = 0; i < count; ++i) obstacles.push_back(obstacleAt(i, i % 2 == 1));
  return Json{{"schema", "xgc2.scene.v1"}, {"id", "projection"}, {"frame", "world"}, {"obstacles", obstacles}};
}
SceneParameters physical() { SceneParameters result; result.profile = "physical"; return result; }

Json command(SceneNode& node, std::string operation, std::string id, Json extra = Json::object()) {
  const auto before = node.envelope();
  extra["operation"] = std::move(operation);
  extra["requestId"] = std::move(id);
  extra["expectedEpoch"] = before.at("epoch");
  extra["expectedRevision"] = before.at("revision");
  return node.commandValue(extra);
}

template<class Message> std::vector<std::uint8_t> bytes(const Message& message) {
  std::vector<std::uint8_t> result(ros::serialization::serializationLength(message));
  ros::serialization::OStream stream(result.data(), result.size());
  ros::serialization::serialize(stream, message);
  return result;
}

void checkProjections(SceneNode& node, Outputs& outputs) {
  const auto envelope = node.envelope();
  const auto& document = envelope.at("document");
  const auto& snapshot = outputs.snapshots.back();
  ASSERT_EQ(snapshot.obstacles.size(), document.at("obstacles").size());
  EXPECT_EQ(snapshot.header.frame_id, document.at("frame"));
  EXPECT_EQ(snapshot.epoch, envelope.at("epoch"));
  EXPECT_EQ(snapshot.revision, envelope.at("revision"));
  for (std::size_t i = 0; i < snapshot.obstacles.size(); ++i)
    EXPECT_EQ(bytes(snapshot.obstacles[i]), bytes(obstacleMessage(document.at("obstacles")[i])));
  node.tick();
  const auto& current = outputs.states.back();
  ASSERT_EQ(current.obstacles.size(), document.at("obstacles").size());
  for (std::size_t i = 0; i < current.obstacles.size(); ++i) {
    const auto value = xgc_scene::state(document.at("obstacles")[i], current.scene_time, current.playing);
    EXPECT_EQ(bytes(current.obstacles[i]), bytes(stateMessage(value)));
  }
  EXPECT_EQ(parseJson(outputs.documents.back()).at("document"), document);
  EXPECT_EQ(outputs.markers_list.back().markers.front().action, visualization_msgs::Marker::DELETEALL);
}

TEST(SceneProjection, ObstacleMessagesPreserveGeometryPoseColorAndMotion) {
  auto definition = obstacle(obstacleAt(1, true));
  const auto body = obstacleMessage(definition);
  EXPECT_EQ(body.id, "o1");
  EXPECT_EQ(body.name, "Obstacle 1");
  EXPECT_EQ(body.motion_type, "constant_twist");
  EXPECT_TRUE(body.dynamic);
  EXPECT_DOUBLE_EQ(body.pose.position.x, 2.0);
  EXPECT_DOUBLE_EQ(body.pose.orientation.w, 1.0);
  ASSERT_EQ(body.parts.size(), 2u);
  EXPECT_EQ(body.parts[0].geometry.type, "box");
  EXPECT_DOUBLE_EQ(body.parts[0].geometry.size.z, 1.0);
  EXPECT_FLOAT_EQ(body.parts[0].color.r, definition.at("parts")[0].at("color")[0].get<float>());
  EXPECT_EQ(body.parts[1].geometry.type, "cylinder");
  EXPECT_DOUBLE_EQ(body.parts[1].geometry.radius, 0.1);
  EXPECT_DOUBLE_EQ(body.parts[1].geometry.height, 1.0);
  EXPECT_TRUE(body.parts[1].geometry.triangles.empty());
}

TEST(SceneProjection, MarkersKeepCapsuleCapsAndConvexTrianglesInThePartFrame) {
  xgc2_geometry_msgs::ScenePart part;
  part.id = "capsule";
  part.geometry.type = "capsule"; part.geometry.radius = 0.25; part.geometry.height = 2.0;
  part.pose.position.x = 3; part.pose.position.y = 4; part.pose.position.z = 5;
  const double norm = std::sqrt(0.3 * 0.3 + 0.5 * 0.5 + 0.2 * 0.2 + 0.79 * 0.79);
  auto& q = part.pose.orientation;
  q.x = 0.3 / norm; q.y = 0.5 / norm; q.z = 0.2 / norm; q.w = 0.79 / norm;
  part.color.r = 1; part.color.a = 0.8;
  const auto markers = partMarkers("o", part, "frame");
  ASSERT_EQ(markers.size(), 3u);
  EXPECT_EQ(markers[0].type, visualization_msgs::Marker::CYLINDER);
  EXPECT_DOUBLE_EQ(markers[0].scale.z, 2);
  for (int i = 1; i <= 2; ++i) {
    const auto& cap = markers[i];
    const double direction = i == 1 ? -1 : 1;
    EXPECT_EQ(cap.id, i);
    EXPECT_EQ(cap.type, visualization_msgs::Marker::SPHERE);
    EXPECT_EQ(cap.header.frame_id, "frame");
    EXPECT_TRUE(cap.header.stamp.isZero());
    EXPECT_EQ(cap.ns, "o/capsule");
    EXPECT_TRUE(cap.frame_locked);
    EXPECT_DOUBLE_EQ(cap.scale.x, 0.5);
    EXPECT_NEAR(cap.pose.position.x, 3 + direction * 2 * (q.x * q.z + q.w * q.y), 1e-14);
    EXPECT_NEAR(cap.pose.position.y, 4 + direction * 2 * (q.y * q.z - q.w * q.x), 1e-14);
    EXPECT_NEAR(cap.pose.position.z, 5 + direction * (1 - 2 * (q.x * q.x + q.y * q.y)), 1e-14);
  }
  part.geometry.height = 0;
  const auto zero = partMarkers("o", part, "frame");
  ASSERT_EQ(zero.size(), 1u);
  EXPECT_EQ(zero[0].id, 1);
  EXPECT_DOUBLE_EQ(zero[0].pose.position.z, 5);
  part.geometry.type = "convex";
  part.geometry.vertices.resize(3);
  part.geometry.vertices[0].x = 1; part.geometry.vertices[1].y = 2; part.geometry.vertices[2].z = 3;
  part.geometry.triangles = {2, 0, 1};
  const auto convex = partMarkers("o", part, "frame");
  ASSERT_EQ(convex.size(), 1u);
  EXPECT_EQ(convex[0].type, visualization_msgs::Marker::TRIANGLE_LIST);
  ASSERT_EQ(convex[0].points.size(), 3u);
  EXPECT_DOUBLE_EQ(convex[0].points[0].z, 3);
  EXPECT_DOUBLE_EQ(convex[0].points[1].x, 1);
  EXPECT_DOUBLE_EQ(convex[0].points[2].y, 2);
}

TEST(SceneNode, CachedProjectionsEqualUncachedResultsAcrossMotionEditsUndoAndDeletion) {
  Outputs outputs;
  double now = 100;
  SceneNode node(scene(), physical(), "/run/scene/", outputs, [&] { return now; });
  checkProjections(node, outputs);
  ASSERT_EQ(command(node, "play", "play").at("success"), true);
  now += 2;
  checkProjections(node, outputs);
  auto edited = node.envelope().at("document").at("obstacles")[0];
  edited["pose"]["position"] = Json::array({9.0, 9.0, 1.0});
  ASSERT_EQ(command(node, "update", "edit", Json{{"obstacle", edited}}).at("success"), true);
  checkProjections(node, outputs);
  edited = node.envelope().at("document").at("obstacles")[2];
  edited["motion"] = Json{{"type", "circle"}, {"center", Json::array({0.0, 0.0, 1.0})},
                         {"radius", 2.0}, {"angular_speed", 0.5}, {"phase", 0.0}};
  edited["pose"]["position"] = Json::array({2.0, 0.0, 1.0});
  ASSERT_EQ(command(node, "update", "motion", Json{{"obstacle", edited}}).at("success"), true);
  checkProjections(node, outputs);
  ASSERT_EQ(command(node, "pause", "pause").at("success"), true);
  now += 1;
  checkProjections(node, outputs);
  ASSERT_EQ(command(node, "delete", "delete", Json{{"id", "o3"}}).at("success"), true);
  checkProjections(node, outputs);
  ASSERT_EQ(command(node, "undo", "undo").at("success"), true);
  checkProjections(node, outputs);
}

TEST(SceneNode, HoldMessagesAreConstantAndOnlyMovingObstaclesGetDynamicTransforms) {
  Outputs outputs;
  double now = 100;
  SceneNode node(scene(), physical(), "/namespace/", outputs, [&] { return now; });
  ASSERT_EQ(outputs.fixed.back().size(), 3u);
  ASSERT_EQ(outputs.moving.back().size(), 2u);
  const auto initial = outputs.states.back();
  const auto base = node.counters();
  ASSERT_EQ(command(node, "play", "play").at("success"), true);
  now += 3;
  node.tick();
  const auto& current = outputs.states.back();
  EXPECT_EQ(bytes(initial.obstacles[0]), bytes(current.obstacles[0]));
  EXPECT_NE(bytes(initial.obstacles[1]), bytes(current.obstacles[1]));
  EXPECT_EQ(node.counters().hold_states, base.hold_states);
  EXPECT_EQ(node.counters().moving_states - base.moving_states, 4u);
  const auto prefix = "xgc_scene_" + digest("/namespace/").substr(0, 12) + "/" + node.envelope().at("epoch").get<std::string>();
  EXPECT_EQ(outputs.fixed.back()[0].child_frame_id, prefix + "/fixed/o0");
  EXPECT_EQ(outputs.moving.back()[0].child_frame_id, prefix + "/moving/o1");
  auto edited = node.envelope().at("document").at("obstacles")[0];
  edited["motion"] = Json{{"type", "constant_twist"}, {"linear", Json::array({1, 0, 0})}};
  ASSERT_EQ(command(node, "update", "start-moving", Json{{"obstacle", edited}}).at("success"), true);
  EXPECT_EQ(outputs.moving.back()[0].child_frame_id, prefix + "/moving/o0");
  edited["motion"] = Json{{"type", "hold"}};
  ASSERT_EQ(command(node, "update", "hold", Json{{"obstacle", edited}}).at("success"), true);
  EXPECT_EQ(outputs.fixed.back()[0].child_frame_id, prefix + "/fixed/o0");
}

TEST(SceneProjection, CacheEqualityPreservesSignedZeroInMessages) {
  auto initial = document(scene(1));
  SceneProjection projection("/run/");
  const auto first = projection.snapshot(initial, "epoch", 1, 100);
  EXPECT_FALSE(std::signbit(first.obstacles[0].pose.position.x));
  initial["obstacles"][0]["pose"]["position"][0] = -0.0;
  const auto changed = projection.snapshot(initial, "epoch", 2, 100);
  EXPECT_TRUE(std::signbit(changed.obstacles[0].pose.position.x));
  EXPECT_EQ(projection.counters().obstacle_messages, 2u);
}

TEST(SceneNode, BatchesNativeApplyAndRebuildsOnlyTheEditedObstacleMessagesAndMarkers) {
  for (const int count : {50, 200, 500}) {
    SCOPED_TRACE(count);
    Outputs outputs;
    auto simulation = std::make_shared<FakeSimulation>();
    SceneParameters config; config.profile = "gazebo";
    SceneNode node(scene(count), config, "/run/", outputs, [] { return 100.0; }, simulation);
    const auto base = node.counters();
    EXPECT_EQ(simulation->applies.size(), 1u);
    EXPECT_EQ(base.snapshot_scans, 1u);
    for (int i = 0; i < 2; ++i) node.tick();
    const auto after_ticks = node.counters();
    EXPECT_LE(after_ticks.state_scans - base.state_scans, 4u);
    EXPECT_EQ(after_ticks.frame_scans, base.frame_scans);
    EXPECT_EQ(after_ticks.obstacle_messages, base.obstacle_messages);
    EXPECT_EQ(after_ticks.part_markers, base.part_markers);
    node.publishDefinition();
    const auto after_definition = node.counters();
    EXPECT_LE(after_definition.definition_scans - after_ticks.definition_scans, 4u);
    EXPECT_EQ(after_definition.obstacle_messages, base.obstacle_messages);
    EXPECT_EQ(after_definition.part_markers, base.part_markers);
    EXPECT_EQ(after_definition.hold_states, base.hold_states);
    auto edited = node.envelope().at("document").at("obstacles")[0];
    edited["name"] = "Edited";
    ASSERT_EQ(command(node, "update", "edit", Json{{"obstacle", edited}}).at("success"), true);
    EXPECT_EQ(simulation->applies.size(), 2u);
    EXPECT_EQ(simulation->applies.back().at("document").at("obstacles").size(), static_cast<std::size_t>(count));
    EXPECT_EQ(node.counters().snapshot_scans - after_definition.snapshot_scans, 1u);
    EXPECT_EQ(node.counters().obstacle_messages - after_definition.obstacle_messages, 1u);
    EXPECT_EQ(node.counters().part_markers - after_definition.part_markers, 2u);
  }
}

TEST(SceneNode, OnlyPublicChangesRepublishDocumentIncludingNativeFailureAndRecovery) {
  Outputs outputs;
  auto simulation = std::make_shared<FakeSimulation>();
  SceneParameters config; config.profile = "gazebo";
  double now = 100;
  SceneNode node(scene(), config, "/run/", outputs, [&] { return now; }, simulation);
  ASSERT_EQ(outputs.documents.size(), 1u);
  for (int i = 0; i < 10; ++i) EXPECT_FALSE(node.publishDocument());
  ASSERT_EQ(command(node, "play", "play").at("success"), true);
  const auto published = outputs.documents.size();
  now += 2;
  node.tick();
  EXPECT_FALSE(node.publishDocument());
  EXPECT_EQ(outputs.documents.size(), published);
  simulation->failure = true;
  auto edited = node.envelope().at("document").at("obstacles")[0];
  edited["name"] = "Rejected";
  const auto failed = command(node, "update", "lost", Json{{"obstacle", edited}});
  EXPECT_EQ(failed.at("success"), true);
  EXPECT_EQ(failed.at("revision"), 2);
  EXPECT_EQ(failed.at("configuration").at("applied").at("known"), true);
  EXPECT_EQ(failed.at("configuration").at("persisted").at("revision"), 1);
  EXPECT_EQ(failed.at("synchronized"), false);
  EXPECT_EQ(failed.at("syncRetryable"), true);
  EXPECT_EQ(outputs.documents.size(), published + 1);
  EXPECT_FALSE(node.publishDocument());
  simulation->failure = false;
  EXPECT_EQ(command(node, "resync", "resync").at("success"), true);
  EXPECT_EQ(outputs.documents.size(), published + 2);
  EXPECT_EQ(node.envelope().at("synchronized"), true);
  EXPECT_EQ(node.envelope().at("syncRetryable"), false);
}

TEST(SceneNode, DuplicateIdsReplayTheOriginalPublicReceiptAfterLaterEdits) {
  Outputs outputs;
  auto simulation = std::make_shared<FakeSimulation>();
  SceneParameters config; config.profile = "gazebo";
  double now = 100;
  SceneNode node(scene(), config, "/run/", outputs, [&] { return now; }, simulation);
  const auto before = node.envelope();
  const Json request{{"operation", "play"}, {"requestId", "play"},
                     {"expectedEpoch", before.at("epoch")}, {"expectedRevision", before.at("revision")}};
  const auto first = node.commandValue(request);
  auto edited = before.at("document").at("obstacles")[0]; edited["name"] = "Later";
  now += 2;
  ASSERT_EQ(command(node, "update", "edit", Json{{"obstacle", edited}}).at("success"), true);
  const auto published = outputs.documents.size();
  EXPECT_EQ(node.commandValue(request), first);
  ASSERT_TRUE(node.requestResult("play"));
  EXPECT_EQ(*node.requestResult("play"), first);
  EXPECT_EQ(outputs.documents.size(), published);
  EXPECT_EQ(simulation->motions.size(), 1u);
  EXPECT_EQ(simulation->applies.size(), 2u);
  auto conflict = request; conflict["operation"] = "pause";
  EXPECT_EQ(node.commandValue(conflict).at("error"), "Request ID was already used for a different operation");
}

TEST(SceneNode, EditsCommitAndRemainReadableWhileNativeReportIsPending) {
  Outputs outputs;
  auto simulation = std::make_shared<FakeSimulation>();
  SceneParameters config; config.profile = "gazebo";
  SceneNode node(scene(), config, "/run/", outputs, [] { return 100.0; }, simulation);
  simulation->hold = true;
  auto edited = node.envelope().at("document").at("obstacles")[0]; edited["name"] = "Edited";
  const auto receipt = command(node, "update", "edit", Json{{"obstacle", edited}});
  EXPECT_EQ(receipt.at("success"), true);
  EXPECT_EQ(receipt.at("synchronized"), false);
  EXPECT_EQ(node.envelope().at("revision"), 2);
  EXPECT_EQ(node.envelope().at("document").at("obstacles")[0].at("name"), "Edited");
  simulation->hold = false;
  node.tick();
  EXPECT_EQ(node.envelope().at("synchronized"), true);
}

TEST(SceneNode, PendingMotionAndUnrestorableNativePhaseNeverAuthorizeSceneTruth) {
  Outputs outputs;
  auto simulation = std::make_shared<FakeSimulation>();
  SceneParameters config; config.profile = "gazebo";
  double now = 100;
  SceneNode node(scene(), config, "/run/", outputs, [&] { return now; }, simulation);
  simulation->hold = true;
  const auto play = command(node, "play", "play");
  EXPECT_EQ(play.at("success"), true);
  EXPECT_EQ(play.at("synchronized"), false);
  const auto status = node.envelope();
  const Json report{{"consumer", "gazebo"}, {"epoch", status.at("epoch")}, {"revision", status.at("revision")},
                    {"applied", true}, {"operational", true}, {"playing", false}, {"scene_time", 0}};
  simulation->pending = report;
  simulation->hold = false;
  node.tick();
  EXPECT_EQ(node.envelope().at("synchronized"), false);
  EXPECT_EQ(node.envelope().at("consumers")[0].at("message"), "World host motion state is pending");
  now += 2;
  simulation->pending = report;
  (*simulation->pending)["playing"] = true;
  node.tick();
  const auto failed = node.envelope();
  EXPECT_EQ(failed.at("synchronized"), false);
  EXPECT_EQ(failed.at("syncRetryable"), true);
  EXPECT_EQ(failed.at("consumers")[0].at("applied"), false);
  EXPECT_EQ(failed.at("consumers")[0].at("message"), "World host scene time differs; native motion seek is unavailable");
}

TEST(SceneNode, PublicReceiptsFollowStoreRetentionAndKeepFailedResults) {
  Outputs outputs;
  SceneNode node(scene(1), physical(), "/run/", outputs, [] { return 100.0; });
  Json latest;
  for (int i = 0; i < 258; ++i) latest = command(node, "unknown", "receipt-" + std::to_string(i));
  EXPECT_EQ(latest.at("success"), false);
  EXPECT_FALSE(node.requestResult("receipt-0"));
  EXPECT_FALSE(node.requestResult("receipt-1"));
  EXPECT_TRUE(node.requestResult("receipt-2"));
  ASSERT_TRUE(node.requestResult("receipt-257"));
  EXPECT_EQ(*node.requestResult("receipt-257"), latest);
  EXPECT_EQ(outputs.documents.size(), 1u);
}

TEST(SceneNode, ParsesPrivateParametersAndRequiresSimulationOnlyForGazebo) {
  Json parameters{{"scene_file", "/scene.yaml"}, {"target_id", "fixture"}, {"xrpc_socket", "/private/scene.sock"},
                  {"profile", "physical"}, {"frozen", true}, {"working_file", "/work.yaml"},
                  {"save_directory", "/save"}, {"generation_parameters_json", "{\"seed\":3}"}};
  const auto parsed = SceneParameters::parse(parameters);
  EXPECT_EQ(parsed.profile, "physical");
  EXPECT_TRUE(parsed.frozen);
  EXPECT_EQ(parsed.working_file, "/work.yaml");
  EXPECT_EQ(parsed.save_directory, "/save");
  EXPECT_EQ(parsed.generation_parameters.at("seed"), 3);
  parameters["profile"] = "gazebo";
  EXPECT_THROW(SceneParameters::parse(parameters), SceneError);
  for (const auto& endpoint : {Json(""), Json("relative/world.sock"), Json(1), Json(std::string("/world\0.sock", 12))}) {
    parameters["simulation_endpoint"] = endpoint;
    try {
      SceneParameters::parse(parameters);
      FAIL() << "Accepted an invalid gazebo endpoint";
    } catch (const SceneError& error) {
      EXPECT_NE(std::string(error.what()).find("~simulation_endpoint"), std::string::npos);
    }
  }
  parameters["simulation_endpoint"] = "/private/0123456789abcdef0123456789abcdef.sock";
  EXPECT_EQ(SceneParameters::parse(parameters).simulation_endpoint, parameters["simulation_endpoint"]);
  for (const auto* profile : {"physical", "xsim"}) {
    parameters["profile"] = profile;
    for (const auto& endpoint : {Json(""), Json("relative/world.sock"), Json(1)}) {
      parameters["simulation_endpoint"] = endpoint;
      EXPECT_TRUE(SceneParameters::parse(parameters).simulation_endpoint.empty());
    }
  }
  parameters["profile"] = "bad";
  EXPECT_THROW(SceneParameters::parse(parameters), SceneError);
  parameters["profile"] = "physical"; parameters["frozen"] = "false";
  EXPECT_THROW(SceneParameters::parse(parameters), SceneError);
}

TEST(SceneNode, ReadinessFactsExposeSourceAndWorkingPathsForEveryProfile) {
  char pattern[] = "/tmp/xgc-scene-facts-XXXXXX";
  const auto directory = ::mkdtemp(pattern);
  ASSERT_NE(directory, nullptr);
  const auto source = std::filesystem::path(directory) / "source.yaml";
  const auto working = std::filesystem::path(directory) / "working.yaml";
  { std::ofstream stream(source); stream << dumpYaml(scene(1)); }
  for (const bool exists : {false, true}) {
    if (exists) { std::ofstream stream(working); stream << dumpYaml(scene(2)); }
    for (const auto* profile : {"physical", "xsim", "gazebo"}) {
      Outputs outputs;
      SceneParameters config;
      config.profile = profile;
      config.scene_file = source;
      config.working_file = working;
      std::shared_ptr<Simulation> simulation;
      if (config.profile == "gazebo") simulation = std::make_shared<FakeSimulation>();
      SceneNode node(config, "/run/", outputs, [] { return 100.0; }, simulation);
      const auto facts = node.peerFacts();
      EXPECT_EQ(facts.at("scene_file"), source.string());
      EXPECT_EQ(facts.at("working_file"), working.string());
      EXPECT_EQ(node.envelope().at("document").at("obstacles").size(), exists ? 2u : 1u);
      EXPECT_EQ(facts.contains("world_host"), config.profile == "gazebo");
    }
  }
  std::filesystem::remove_all(directory);
}

TEST(SceneNode, ShutdownPublishesOfflineOnceAndStopsFurtherTicks) {
  Outputs outputs;
  auto simulation = std::make_shared<FakeSimulation>();
  SceneParameters config; config.profile = "gazebo";
  SceneNode node(scene(), config, "/run/", outputs, [] { return 100.0; }, simulation);
  const auto states = outputs.states.size();
  node.shutdown(); node.shutdown(); node.tick();
  EXPECT_FALSE(node.online());
  EXPECT_EQ(outputs.states.size(), states);
  EXPECT_EQ(simulation->closes, 1);
  EXPECT_EQ(outputs.documents.size(), 2u);
  EXPECT_EQ(parseJson(outputs.documents.back()).at("online"), false);
}

}  // namespace
}  // namespace xgc_scene
