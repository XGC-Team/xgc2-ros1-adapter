#include "../geometry_wire.hpp"
#include "xgc_scene/lidar_ros/cloud.hpp"
#include "xgc_scene/lidar_ros/parameters.hpp"
#include "xgc_scene/lidar_ros/scene_tap.hpp"
#include "xgc_scene/node.hpp"
#include "xgc_scene/document.hpp"

#include <gtest/gtest.h>
#include <ros/serialization.h>
#include <cstring>
#include <sstream>

namespace xgc_scene::lidar_ros {
namespace {
constexpr std::int64_t second = 1000000000;
Json parameters() { return {{"lidar_xrpc_socket", "/tmp/lf-edge.sock"}}; }
Json manifest(std::size_t sensors = 1) {
  Json robots = Json::array();
  for (std::size_t i = 0; i < sensors; ++i)
    robots.push_back({{"namespace", "/A" + std::to_string(i)}, {"hRes", 1}, {"vRes", 1},
                     {"hFovDeg", 90}, {"vFovDeg", 0}, {"publishBeams", true}});
  return {{"schemaVersion", 1}, {"robots", robots}, {"referenceCloud", {{"surfaceSpacing", 0.5}}}};
}

struct Outputs : RosOutputs {
  std::size_t snapshots = 0, states = 0;
  void snapshot(const xgc2_geometry_msgs::SceneSnapshot&) override { ++snapshots; }
  void state(const xgc2_geometry_msgs::SceneState&) override { ++states; }
  void document(const std::string&) override {}
  void markers(const visualization_msgs::MarkerArray&) override {}
  void staticTransforms(const std::vector<geometry_msgs::TransformStamped>&) override {}
  void dynamicTransforms(const std::vector<geometry_msgs::TransformStamped>&) override {}
};
struct Sink : lidar::Sink {
  explicit Sink(std::size_t n = 1) : points(n, cloudLayout()), beams(n, cloudLayout(true)), counts(n) {}
  std::vector<sensor_msgs::PointCloud2> points, beams;
  std::vector<unsigned> counts;
  sensor_msgs::PointCloud2 reference = cloudLayout();
  std::vector<lidar::ReferenceAction> actions;
  void onPoints(std::size_t i, lidar::CloudView cloud) override { fillCloud(points[i], cloud); ++counts[i]; }
  void onBeams(std::size_t i, lidar::CloudView cloud) override { fillCloud(beams[i], cloud); }
  void onReference(lidar::CloudView cloud, lidar::ReferenceAction action) override {
    fillCloud(reference, cloud);
    actions.push_back(action);
  }
};

xgc2_geometry_msgs::SceneSnapshot snapshot(bool dynamic = false, std::string epoch = "first", std::uint64_t revision = 1) {
  xgc2_geometry_msgs::SceneSnapshot message;
  message.header.frame_id = "world";
  message.epoch = std::move(epoch);
  message.revision = revision;
  xgc2_geometry_msgs::SceneObstacle obstacle;
  obstacle.id = "wall";
  obstacle.dynamic = dynamic;
  obstacle.pose.position.x = 4;
  obstacle.pose.orientation.w = 1;
  xgc2_geometry_msgs::ScenePart part;
  part.pose.orientation.w = 1;
  part.geometry = geometry_test::wire(geometry_test::Geometry::box({1, 4, 2}));
  obstacle.parts.push_back(part);
  message.obstacles.push_back(obstacle);
  return message;
}
xgc2_geometry_msgs::SceneState state(const xgc2_geometry_msgs::SceneSnapshot& definition, double x = 4) {
  xgc2_geometry_msgs::SceneState message;
  message.header.frame_id = "world";
  message.epoch = definition.epoch;
  message.revision = definition.revision;
  xgc2_geometry_msgs::SceneObstacleState obstacle;
  obstacle.id = "wall";
  obstacle.pose = definition.obstacles[0].pose;
  obstacle.pose.position.x = x;
  message.obstacles.push_back(obstacle);
  return message;
}

Json sessionDocument(std::size_t sensors = 1) {
  Json robots = Json::array();
  for (std::size_t i = 0; i < sensors; ++i)
    robots.push_back({{"namespace", "/A" + std::to_string(i)},
        {"binding", {{"simulationSensors", {{"simpleLidar", {{"enabled", true},
            {"mode", "depth_frustum"}, {"acceleration", "cpu"}, {"hRes", 3}, {"vRes", 3}}}}}}}});
  return {{"runMode", "simulation"}, {"robots", robots},
      {"visualizationTopics", Json::array({{{"topic", "/xgc/scene/reference_cloud"},
          {"messageType", "sensor_msgs/PointCloud2"}, {"role", "semantic"}, {"visible3d", true}}})}};
}

TEST(LidarSession, StdinEmptyValidSizeJsonRunModesAndReadFailures) {
  std::istringstream empty;
  EXPECT_FALSE(readSessionInput(empty));
  auto document = sessionDocument();
  document["robots"][0]["asset"] = {{"opaque", "unchanged"}};
  for (const auto* mode : {"simulation", "hybrid", "physical"}) {
    document["runMode"] = mode;
    std::istringstream stream(document.dump());
    auto input = readSessionInput(stream);
    ASSERT_TRUE(input);
    EXPECT_EQ(input->run_mode, mode);
    EXPECT_EQ(input->robots, document["robots"]);
    EXPECT_EQ(input->visualization_topics, document["visualizationTopics"]);
  }
  auto maximum = document.dump();
  maximum.resize(kMaxDocumentBytes, ' ');
  std::istringstream boundary(maximum);
  EXPECT_TRUE(readSessionInput(boundary));
  maximum.push_back(' ');
  std::istringstream oversized(maximum);
  try { readSessionInput(oversized); FAIL(); }
  catch (const SceneError& error) {
    EXPECT_NE(std::string(error.what()).find("--session-stdin"), std::string::npos);
    EXPECT_NE(std::string(error.what()).find("exceeds 8 MiB"), std::string::npos);
  }
  for (const auto& invalid : {std::string("{"), std::string("[]"),
      std::string("{\"runMode\":\"invalid\",\"robots\":[],\"visualizationTopics\":[]}"),
      std::string("{\"robots\":[],\"visualizationTopics\":[]}")}) {
    std::istringstream stream(invalid);
    try { readSessionInput(stream); FAIL(); }
    catch (const SceneError& error) {
      EXPECT_NE(std::string(error.what()).find("--session-stdin"), std::string::npos);
      EXPECT_NE(std::string(error.what()).find("runMode"), std::string::npos);
    }
  }
  std::istringstream failed;
  failed.setstate(std::ios::badbit);
  try { readSessionInput(failed); FAIL(); }
  catch (const SceneError& error) {
    EXPECT_NE(std::string(error.what()).find("--session-stdin: read failed"), std::string::npos);
  }
}

TEST(LidarParameters, DefaultsProfilesReferenceOnlyAndNamedErrors) {
  auto values = parameters();
  auto empty = Parameters::parse(values, "gazebo");
  EXPECT_FALSE(empty.have_fleet);
  EXPECT_EQ(empty.facts("gazebo"), (Json{{"profile", "gazebo"}, {"sensors", 0}, {"backends", Json::array({"auto", "cpu", "gpu"})}}));
  const auto session = lidar::parseSessionInput(sessionDocument().dump());
  values["lidar_enabled"] = false;
  values["lidar_defaults_json"] = Json{{"min_range", 0.2}, {"noise_std", 0.01}, {"seed", 42},
      {"yaw_only", true}, {"pose_timeout", 0.8}, {"worker_threads", 2}, {"vehicle_bodies", true},
      {"vehicle_body_radius", 0.4}}.dump();
  const auto fleet = Parameters::parse(values, "xsim", session, true);
  ASSERT_EQ(fleet.manifest.robots.size(), 1u);
  EXPECT_FALSE(fleet.enabled);
  EXPECT_EQ(fleet.manifest.robots[0].config.min_range, 0.2);
  EXPECT_EQ(fleet.manifest.robots[0].config.noise_std, 0.01);
  EXPECT_EQ(fleet.manifest.robots[0].config.seed, 42u);
  EXPECT_TRUE(fleet.options.yaw_only);
  EXPECT_EQ(fleet.options.pose_timeout, 0.8);
  EXPECT_EQ(fleet.options.worker_threads, 2);
  EXPECT_TRUE(fleet.options.vehicle_bodies);
  EXPECT_EQ(fleet.options.vehicle_body_radius, 0.4);
  for (const auto* profile : {"xsim", "physical"}) {
    const auto absent = Parameters::parse(values, profile, session);
    EXPECT_FALSE(absent.have_fleet);
    EXPECT_TRUE(absent.manifest.robots.empty());
    EXPECT_EQ(absent.facts(profile), (Json{{"profile", profile}, {"sensors", 0}, {"backends", Json::array()}}));
  }
  EXPECT_TRUE(Parameters::parse(values, "gazebo", lidar::parseSessionInput(sessionDocument(0).dump())).have_fleet);
  auto no_sensors = sessionDocument(0);
  no_sensors["visualizationTopics"] = Json::array();
  EXPECT_FALSE(Parameters::parse(values, "gazebo", lidar::parseSessionInput(no_sensors.dump())).have_fleet);
  const std::pair<const char*, Json> errors[] = {
      {"lidar_defaults_json", "[]"}, {"lidar_defaults_json", "{\"preset\":\"test\"}"},
      {"lidar_defaults_json", "{\"seed\":-1}"}, {"lidar_defaults_json", "{\"h_res\":1.5}"},
      {"lidar_defaults_json", "{\"pose_timeout\":0}"},
      {"lidar_enabled", "true"}, {"lidar_xrpc_socket", "relative"}};
  for (const auto& test : errors) {
    auto invalid = parameters();
    invalid[test.first] = test.second;
    try { Parameters::parse(invalid, "gazebo", session); FAIL() << test.first; }
    catch (const SceneError& error) { EXPECT_NE(std::string(error.what()).find(test.first), std::string::npos); }
  }
  values["lidar_defaults_json"] = "{\"typo\":1}";
  try { Parameters::parse(values, "gazebo"); FAIL(); }
  catch (const SceneError& error) { EXPECT_NE(std::string(error.what()).find("unknown key 'typo'"), std::string::npos); }
  for (const auto* backend : {"auto", "cpu", "gpu"}) {
    auto accepted = sessionDocument();
    accepted["robots"][0]["binding"]["simulationSensors"]["simpleLidar"]["acceleration"] = backend;
    const auto input = lidar::parseSessionInput(accepted.dump());
    const auto configured = Parameters::parse(parameters(), "xsim", input, true);
    ASSERT_EQ(configured.manifest.robots.size(), 1u);
    EXPECT_EQ(configured.manifest.robots[0].backend, backend);
    EXPECT_TRUE(Parameters::parse(parameters(), "gazebo", input).manifest.robots.empty());
  }
  for (int refusal = 0; refusal < 3; ++refusal) {
    auto invalid = sessionDocument();
    auto& sensor = invalid["robots"][0]["binding"]["simulationSensors"]["simpleLidar"];
    const char* message;
    if (refusal == 0) { sensor["acceleration"] = "invalid-backend"; message = "invalid authored simpleLidar.acceleration"; }
    else if (refusal == 1) { invalid["robots"][0]["namespace"] = "invalid namespace"; message = "invalid lidar namespace"; }
    else { sensor["mode"] = "penetrating"; sensor["publishBeams"] = true; message = "penetrating sampled clouds have no beams"; }
    const auto input = lidar::parseSessionInput(invalid.dump());
    try { Parameters::parse(parameters(), "xsim", input, true); FAIL(); }
    catch (const SceneError& error) { EXPECT_NE(std::string(error.what()).find(message), std::string::npos); }
    EXPECT_TRUE(Parameters::parse(parameters(), "gazebo", input).manifest.robots.empty());
    EXPECT_FALSE(Parameters::parse(parameters(), "xsim", input).have_fleet);
  }
}

TEST(LidarDemand, PointsAndBeamsConnectDisconnectWithoutPolling) {
  Demand demand;
  EXPECT_FALSE(demand.change(false, false));
  EXPECT_TRUE(demand.change(false, true));
  EXPECT_TRUE(demand.change(false, true));
  EXPECT_TRUE(demand.change(true, true));
  EXPECT_TRUE(demand.change(false, false));
  EXPECT_TRUE(demand.change(false, false));
  EXPECT_FALSE(demand.change(true, false));
  EXPECT_EQ(demand.points, 0u);
  EXPECT_EQ(demand.beams, 0u);
}

TEST(LidarCloud, WireLayoutExactPoseStampAndReusedStorage) {
  for (const bool beams : {false, true}) {
    auto message = cloudLayout(beams);
    const unsigned step = beams ? 32 : 12;
    const std::vector<std::uint8_t> bytes(3 * step, 0x42);
    fillCloud(message, {bytes.data(), 3, second + 12345, step, 1});
    const char* fields[] = {"x", "y", "z", "dx", "dy", "dz", "range", "hit"};
    ASSERT_EQ(message.fields.size(), step / 4);
    for (std::size_t i = 0; i < message.fields.size(); ++i) {
      EXPECT_EQ(message.fields[i].name, fields[i]);
      EXPECT_EQ(message.fields[i].offset, 4 * i);
      EXPECT_EQ(message.fields[i].datatype, sensor_msgs::PointField::FLOAT32);
      EXPECT_EQ(message.fields[i].count, 1u);
    }
    EXPECT_EQ(message.header.frame_id, "world");
    EXPECT_EQ(message.header.stamp.toNSec(), second + 12345);
    EXPECT_EQ(message.point_step, step);
    EXPECT_EQ(message.height, 1u);
    EXPECT_EQ(message.width, 3u);
    EXPECT_EQ(message.row_step, 3 * step);
    EXPECT_TRUE(message.is_dense);
    EXPECT_FALSE(message.is_bigendian);
    EXPECT_EQ(message.data, bytes);
    auto* storage = message.data.data();
    auto* layout = message.fields.data();
    for (int i = 0; i < 100; ++i) {
      fillCloud(message, {bytes.data(), 2, second + i, step, 1});
      EXPECT_EQ(message.data.data(), storage);
      EXPECT_EQ(message.fields.data(), layout);
    }
    fillCloud(message, {nullptr, 0, second, step, 2});
    EXPECT_EQ(message.width, 0u);
    EXPECT_EQ(message.row_step, 0u);
    EXPECT_TRUE(message.data.empty());
  }
}

TEST(LidarSceneTap, AppliedPairEpochDynamicFramesPendingAndRefusedTruth) {
  Outputs outputs;
  Sink sink;
  lidar::Engine engine(lidar::parseManifest(manifest().dump()), sink);
  engine.setReferenceDemand(true);
  SceneTap tap(outputs, &engine, [] { return second; });
  auto definition = snapshot(true);
  tap.application("first", 1, true);
  tap.snapshot(definition);
  EXPECT_FALSE(engine.sceneReady());
  auto current = state(definition);
  current.epoch = "other";
  tap.state(current);
  EXPECT_FALSE(engine.sceneReady());
  current.epoch = "first";
  tap.state(current);
  ASSERT_TRUE(engine.sceneReady());
  EXPECT_EQ(engine.sensorScene(0)->closest({0, 0, 0}, {1, 0, 0}, 20), 3.5);
  auto first = engine.sensorScene(0);
  engine.tick(second);
  EXPECT_EQ(sink.actions.back(), lidar::ReferenceAction::Publish);
  const auto revision = engine.sensorScene(0);
  tap.state(current);
  EXPECT_EQ(engine.sensorScene(0), revision);
  current.obstacles[0].pose.position.x = 6;
  tap.state(current);
  EXPECT_NE(engine.sensorScene(0), first);
  EXPECT_EQ(engine.sensorScene(0)->closest({0, 0, 0}, {1, 0, 0}, 20), 5.5);
  tap.application("first", 2, false);
  EXPECT_FALSE(engine.sceneReady());
  engine.tick(2 * second);
  EXPECT_EQ(sink.actions.back(), lidar::ReferenceAction::Clear);
  EXPECT_TRUE(sink.reference.data.empty());
  definition.revision = 2;
  tap.snapshot(definition);
  tap.state(state(definition));
  EXPECT_FALSE(engine.sceneReady());
  tap.application("first", 2, true);
  ASSERT_TRUE(engine.sceneReady());
  tap.application("first", 2, false);
  EXPECT_FALSE(engine.sceneReady());
  tap.application("second", 1, true);
  definition.epoch = "second";
  definition.revision = 1;
  tap.snapshot(definition);
  EXPECT_FALSE(engine.sceneReady());
  current = state(definition);
  current.obstacles.clear();
  tap.state(current);
  EXPECT_FALSE(engine.sceneReady());
  EXPECT_EQ(tap.error(), "state lacks dynamic obstacle 'wall'");
  tap.state(state(definition));
  EXPECT_TRUE(engine.sceneReady());
  current = state(definition);
  current.header.frame_id = "map";
  tap.state(current);
  EXPECT_FALSE(engine.sceneReady());
  tap.state(state(definition));
  EXPECT_TRUE(engine.sceneReady());
  definition.revision = 2;
  definition.header.frame_id = "map";
  tap.snapshot(definition);
  EXPECT_FALSE(engine.sceneReady());
  EXPECT_EQ(tap.error(), "snapshot frame must be world");
}

TEST(LidarSceneTap, StaticConversionRefusalAndOldDefinitionIgnored) {
  Outputs outputs;
  Sink sink;
  lidar::Engine engine(lidar::parseManifest(manifest().dump()), sink);
  SceneTap tap(outputs, &engine, [] { return second; });
  auto definition = snapshot();
  tap.application("first", 1, true);
  tap.snapshot(definition);
  ASSERT_TRUE(engine.sceneReady());
  auto installed = engine.sensorScene(0);
  definition.obstacles.clear();
  tap.snapshot(definition);
  EXPECT_EQ(engine.sensorScene(0), installed);
  definition = snapshot(false, "first", 2);
  definition.obstacles[0].parts[0].geometry.parameters.clear();
  tap.application("first", 2, true);
  tap.snapshot(definition);
  EXPECT_FALSE(engine.sceneReady());
  EXPECT_FALSE(tap.error().empty());
  definition = snapshot(false, "replacement", 1);
  definition.obstacles.clear();
  tap.application("replacement", 1, true);
  tap.snapshot(definition);
  EXPECT_TRUE(engine.sceneReady());
  EXPECT_EQ(engine.sensorScene(0)->obstacleCount(), 0u);
}

TEST(LidarEdgeWorkers, TwoSensorConcurrentMessagesAndObservationContract) {
  Sink sink(2);
  lidar::EngineOptions options;
  options.worker_threads = 2;
  lidar::Engine engine(lidar::parseManifest(manifest(2).dump()), sink, options);
  engine.setScene({xgc2_math::scene::Obstacle::box({4, 0, 0}, {1, 4, 2})}, 1);
  xgc2_math::Pose3 pose;
  EXPECT_EQ(engine.tick(second), 0u);
  for (std::size_t i = 0; i < 2; ++i) engine.setDemand(i, true);
  for (int t = 0; t < 100; ++t) {
    const auto now = second + t * second / 10;
    for (std::size_t i = 0; i < 2; ++i) engine.setPose(i, pose, now - 12345);
    EXPECT_EQ(engine.tick(now), 2u);
    for (std::size_t i = 0; i < 2; ++i) {
      EXPECT_EQ(sink.points[i].header.stamp.toNSec(), now - 12345);
      EXPECT_EQ(sink.points[i].width, 1u);
      EXPECT_EQ(sink.beams[i].point_step, 32u);
      float x;
      std::memcpy(&x, sink.points[i].data.data(), sizeof x);
      EXPECT_FLOAT_EQ(x, 3.5);
    }
  }
  EXPECT_FALSE(engine.setPose(0, pose, 11 * second, "map"));
  EXPECT_EQ(engine.tick(11 * second), 1u);
  pose.orientation.coeffs().setZero();
  EXPECT_FALSE(engine.setPose(1, pose, 12 * second));
  EXPECT_EQ(engine.tick(12 * second), 0u);
  engine.setEnabled(false);
  EXPECT_EQ(engine.tick(13 * second), 0u);
}

TEST(LidarSceneTap, NativeSceneNodeReportsAreTheApplicationBoundary) {
  struct Simulation : xgc_scene::Simulation {
    lidar::Engine* engine;
    bool refuse = false;
    std::optional<Json> report;
    void update(const Json&, const std::string& epoch, std::uint64_t revision, const Json&) override {
      EXPECT_FALSE(engine->sceneReady());
      report = Json{{"epoch", epoch}, {"revision", revision}, {"applied", !refuse},
                    {"message", refuse ? "refused native application" : "applied"}};
    }
    std::optional<Json> takeReport() override { auto result = std::move(report); report.reset(); return result; }
    void close() override {}
  };
  Outputs outputs;
  Sink sink;
  lidar::Engine engine(lidar::parseManifest(manifest().dump()), sink);
  SceneTap tap(outputs, &engine, [] { return second; });
  auto simulation = std::make_shared<Simulation>();
  simulation->engine = &engine;
  SceneParameters config;
  config.profile = "gazebo";
  Json document{{"schema", "xgc2.scene.v1"}, {"id", "truth"}, {"frame", "world"}, {"obstacles", Json::array()}};
  SceneNode node(document, config, "/test/", tap, [] { return 1.; }, simulation);
  EXPECT_TRUE(engine.sceneReady());
  auto before = node.envelope();
  simulation->refuse = true;
  Json command{{"operation", "add"}, {"requestId", "refused"}, {"expectedEpoch", before.at("epoch")},
      {"expectedRevision", before.at("revision")}, {"obstacle", {{"id", "box"},
          {"parts", Json::array({{{"id", "p"}, {"geometry", {{"type", "sphere"}, {"radius", 1}}}}})}}}};
  const auto result = node.commandValue(command);
  EXPECT_FALSE(result.at("synchronized").get<bool>());
  EXPECT_FALSE(engine.sceneReady());
  node.tick();
  EXPECT_FALSE(engine.sceneReady());
  simulation->refuse = false;
  before = node.envelope();
  command = {{"operation", "resync"}, {"requestId", "retry"}, {"expectedEpoch", before.at("epoch")},
      {"expectedRevision", before.at("revision")}};
  node.commandValue(command);
  EXPECT_TRUE(engine.sceneReady());
  node.gazeboFailure(before.at("epoch"), before.at("revision"), "native lost synchronization");
  EXPECT_FALSE(engine.sceneReady());
}
}  // namespace
}  // namespace xgc_scene::lidar_ros
