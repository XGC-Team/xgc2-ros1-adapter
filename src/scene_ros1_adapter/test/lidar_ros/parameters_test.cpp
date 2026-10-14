#include "xgc_scene/lidar_ros/parameters.hpp"

#include <gtest/gtest.h>

namespace xgc_scene::lidar_ros {
namespace {
Json values() { return {{"lidar_xrpc_socket", "/tmp/parameter-fixture.sock"}}; }
Json sessionDocument() {
  return {{"runMode", "simulation"},
      {"robots", Json::array({{{"namespace", "/robot"}, {"binding", {
          {"hybridSource", "simulation"}, {"simulationSensors", {{"simpleLidar", {
              {"enabled", true}, {"mode", "raycast"}, {"acceleration", "cpu"},
              {"rateHz", 4}, {"rangeMeters", 15}, {"hFovDeg", 270}, {"vFovDeg", 45},
              {"hRes", 64}, {"vRes", 4}}}}}}}}})},
      {"visualizationTopics", Json::array({{{"topic", "/xgc/scene/reference_cloud"},
          {"messageType", "sensor_msgs/PointCloud2"}, {"role", "semantic"}, {"visibleAr", true}}})}};
}

TEST(LidarProfileParameters, GazeboRetainsOnlyReferenceForEveryBackendAndLegacyMode) {
  for (const char* run_mode : {"simulation", "hybrid"}) {
    for (const char* backend : {"cpu", "gpu", "auto"}) {
      for (const char* mode : {"raycast", "penetrating", "depth_frustum"}) {
        for (bool point_cloud : {false, true}) {
          auto document = sessionDocument();
          document["runMode"] = run_mode;
          auto& sensor = document["robots"][0]["binding"]["simulationSensors"]["simpleLidar"];
          sensor["acceleration"] = backend;
          sensor["mode"] = mode;
          sensor["preset"] = "bridge_equivalent";
          sensor["publishBeams"] = true;
          const auto input = lidar::parseSessionInput(document.dump());
          const auto result = Parameters::parse(values(), "gazebo", input, point_cloud);
          EXPECT_TRUE(result.manifest.robots.empty());
          EXPECT_TRUE(result.manifest.bodies.empty());
          EXPECT_FALSE(result.manifest.body_roster);
          ASSERT_TRUE(result.manifest.reference);
          EXPECT_EQ(result.manifest.reference->surface_spacing, 0.1);
          EXPECT_TRUE(result.have_fleet);
          EXPECT_EQ(input.robots, document["robots"]);
          EXPECT_EQ(input.facts, document);
        }
      }
    }
  }
}

TEST(LidarProfileParameters, GazeboRobotSelectionCannotCreateFleetWithoutVisibleReference) {
  auto document = sessionDocument();
  document["visualizationTopics"] = Json::array();
  for (bool point_cloud : {false, true}) {
    const auto result = Parameters::parse(values(), "gazebo", lidar::parseSessionInput(document.dump()), point_cloud);
    EXPECT_FALSE(result.have_fleet);
    EXPECT_TRUE(result.manifest.robots.empty());
    EXPECT_FALSE(result.manifest.reference);
  }
  document = sessionDocument();
  document["runMode"] = "physical";
  EXPECT_FALSE(Parameters::parse(values(), "gazebo", lidar::parseSessionInput(document.dump()), true).have_fleet);
}

TEST(LidarProfileParameters, XSimSavedCloudPreservesRobotScanAndReference) {
  const auto input = lidar::parseSessionInput(sessionDocument().dump());
  const auto result = Parameters::parse(values(), "xsim", input, true);
  ASSERT_EQ(result.manifest.robots.size(), 1u);
  const auto& sensor = result.manifest.robots.front();
  EXPECT_EQ(sensor.name_space, "/robot");
  EXPECT_EQ(sensor.backend, "cpu");
  EXPECT_EQ(sensor.rate_hz, 4);
  EXPECT_EQ(sensor.config.range, 15);
  EXPECT_EQ(sensor.config.h_fov_deg, 270);
  EXPECT_EQ(sensor.config.v_fov_deg, 45);
  EXPECT_EQ(sensor.config.h_res, 64);
  EXPECT_EQ(sensor.config.v_res, 4);
  ASSERT_TRUE(result.manifest.reference);
  EXPECT_EQ(result.manifest.reference->surface_spacing, 0.1);
  EXPECT_TRUE(result.have_fleet);
  const auto absent = Parameters::parse(values(), "xsim", input, false);
  EXPECT_FALSE(absent.have_fleet);
  EXPECT_TRUE(absent.manifest.robots.empty());
  EXPECT_FALSE(absent.manifest.reference);
}
} // namespace
} // namespace xgc_scene::lidar_ros
