#include "xgc_scene/lidar/manifest.hpp"

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

using namespace xgc_scene::lidar;
using Json = nlohmann::json;

namespace {
Json manifest() {
  return {{"schemaVersion", 1},
          {"robots", Json::array({{{"namespace", "/A"}}})}};
}
void refused(const Json &doc, const std::string &message) {
  try {
    parseManifest(doc.dump());
    FAIL() << "accepted " << doc;
  } catch (const std::exception &e) {
    EXPECT_EQ(message, e.what());
  }
}
} // namespace

TEST(Manifest, DefaultsAndAllSensorFields) {
  auto doc = manifest();
  auto fleet = parseManifest(doc.dump());
  ASSERT_EQ(1u, fleet.robots.size());
  EXPECT_EQ("/vrpn_client_node/A/pose", fleet.robots[0].pose_topic);
  EXPECT_EQ(1, fleet.robots[0].body_id);
  EXPECT_EQ(10.0, fleet.robots[0].rate_hz);
  EXPECT_FALSE(fleet.body_roster);
  doc["robots"][0].update({{"mode", "depth_frustum"},
                           {"rangeMeters", 8},
                           {"minRangeMeters", 0.1},
                           {"hFovDeg", 90},
                           {"vFovDeg", 60},
                           {"hRes", 40},
                           {"vRes", 20},
                           {"surfaceSpacing", 0.2},
                           {"keepBuried", true},
                           {"headingCrop", true},
                           {"headingCosMin", 0.5},
                           {"verticalSlabTan", 0.7},
                           {"publishBeams", true},
                           {"rateHz", 25},
                           {"poseTopic", "/explicit/pose"}});
  xgc2_math::scene::SensorConfig defaults;
  defaults.noise_std = 0.01;
  defaults.seed = 42;
  fleet = parseManifest(doc.dump(), defaults);
  const auto &s = fleet.robots[0];
  EXPECT_EQ("/explicit/pose", s.pose_topic);
  EXPECT_EQ(40, s.config.width);
  EXPECT_EQ(20, s.config.height);
  EXPECT_DOUBLE_EQ(20.0, s.config.fx);
  EXPECT_EQ(8.0, s.config.range);
  EXPECT_EQ(0.1, s.config.min_range);
  EXPECT_EQ(0.2, s.config.surface_spacing);
  EXPECT_EQ(0.01, s.config.noise_std);
  EXPECT_EQ(42u, s.config.seed);
  EXPECT_TRUE(s.config.penetrating_keep_buried);
  EXPECT_TRUE(s.config.penetrating_heading_crop);
  EXPECT_EQ(0.5, s.config.heading_cos_min);
  EXPECT_EQ(0.7, s.config.vertical_slab_tan);
  EXPECT_EQ(25.0, s.rate_hz);
  EXPECT_TRUE(s.publish_beams);
}

TEST(Manifest, OriginalRefusalMessages) {
  auto d = manifest();
  d["schemaVersion"] = 2;
  refused(d, "unsupported fleet sensor schema");
  d = manifest();
  d["robots"] = Json::array();
  refused(d, "fleet needs 1..512 robots or a scene reference cloud");
  for (const auto &ns : {"A", "/1A", "/A-", "/A/", "/"}) {
    d = manifest();
    d["robots"][0]["namespace"] = ns;
    refused(d, "invalid or duplicated fleet namespace");
  }
  d = manifest();
  d["robots"].push_back(d["robots"][0]);
  refused(d, "invalid or duplicated fleet namespace");
  d = manifest();
  d["robots"][0]["mode"] = "other";
  refused(d, "invalid fleet sensor mode");
  for (const double rate : {0.0, -1.0, 100.1}) {
    d = manifest();
    d["robots"][0]["rateHz"] = rate;
    refused(d, "invalid fleet sensor rate");
  }
  for (const double spacing : {0.0, -0.1, 10.1}) {
    d = manifest();
    d["referenceCloud"] = {{"surfaceSpacing", spacing}};
    refused(d, "invalid reference map spacing");
  }
  d = manifest();
  d["robots"][0]["rangeMeters"] = 0;
  refused(d, "range must be > 0");
  d = manifest();
  d["robots"][0]["surfaceSpacing"] = 0;
  refused(d, "scene sample spacing must be finite and > 0");
  EXPECT_THROW(parseManifest(""), std::invalid_argument);
  EXPECT_THROW(parseManifest(std::string(kMaxManifestBytes + 1, ' ')),
               std::invalid_argument);
  EXPECT_THROW(parseManifest("{"), std::exception);
  EXPECT_THROW(parseManifest("{}"), std::exception);
}

TEST(Manifest, ReferenceOnlyAndLargeFleetLimits) {
  auto d = manifest();
  d["robots"] = Json::array();
  d["referenceCloud"] = {{"surfaceSpacing", 10.0}};
  EXPECT_TRUE(parseManifest(d.dump()).robots.empty());
  d.erase("referenceCloud");
  for (std::size_t i = 0; i < 512; ++i)
    d["robots"].push_back(
        {{"namespace",
          "/large_fleet_with_long_namespace/uav_" + std::to_string(i)},
         {"mode", "penetrating"},
         {"rangeMeters", 8},
         {"rateHz", 20},
         {"hFovDeg", 360},
         {"vFovDeg", 180},
         {"hRes", 360},
         {"vRes", 32},
         {"surfaceSpacing", 0.1},
         {"keepBuried", true},
         {"headingCrop", false},
         {"headingCosMin", 0},
         {"verticalSlabTan", 0},
         {"publishBeams", false}});
  const std::string raw = d.dump();
  EXPECT_GT(raw.size(), 131072u);
  EXPECT_EQ(512u, parseManifest(raw).robots.size());
  EXPECT_EQ(512u, parseManifest(
                      raw + std::string(kMaxManifestBytes - raw.size(), ' '))
                      .robots.size());
  d["robots"].push_back({{"namespace", "/extra"}});
  refused(d, "fleet needs 1..512 robots or a scene reference cloud");
}

TEST(Manifest, BodyRosterIdentityGeometryAndPoseSources) {
  auto d = manifest();
  d["bodies"] = Json::array(
      {{{"id", 41},
        {"namespace", "/A"},
        {"poseTopic", "/truth/A"},
        {"geometry", {{"type", "sphere"}, {"radiusMeters", 0.3}}}}});
  auto f = parseManifest(d.dump());
  EXPECT_TRUE(f.body_roster);
  EXPECT_EQ(41, f.robots[0].body_id);
  EXPECT_EQ("/truth/A", f.robots[0].pose_topic);
  d["robots"][0]["poseTopic"] = "/measurement/A";
  EXPECT_EQ("/measurement/A", parseManifest(d.dump()).robots[0].pose_topic);
  d["robots"][0]["mode"] = "penetrating";
  EXPECT_EQ("/measurement/A", parseManifest(d.dump()).robots[0].pose_topic);
  d["robots"][0].erase("poseTopic");
  EXPECT_EQ("/vrpn_client_node/A/pose",
            parseManifest(d.dump()).robots[0].pose_topic);
  d["robots"][0]["bodyId"] = 7;
  refused(d, "sensor bodyId disagrees with its world member");
  d["robots"][0].erase("bodyId");
  d["robots"][0]["namespace"] = "/missing";
  refused(d, "sensor namespace is not a world body member");
  d["robots"][0]["namespace"] = "/A";
  const auto original = d;
  for (const auto &change :
       {Json{{"id", -1}}, Json{{"namespace", "/bad-"}},
        Json{{"poseTopic", "relative"}},
        Json{{"geometry", {{"type", "box"}, {"radiusMeters", 0.3}}}},
        Json{{"geometry", {{"type", "sphere"}, {"radiusMeters", 0}}}}}) {
    d = original;
    d["bodies"][0].update(change);
    refused(d, "invalid body id/namespace/truth topic/sphere geometry");
  }
  d = original;
  d["bodies"].push_back(d["bodies"][0]);
  refused(d, "invalid body id/namespace/truth topic/sphere geometry");
  d = original;
  d["bodies"] = Json::array();
  for (int i = 0; i < 512; ++i)
    d["bodies"].push_back(
        {{"id", i},
         {"namespace", i == 0 ? "/A" : "/B" + std::to_string(i)},
         {"poseTopic", "/truth/B" + std::to_string(i)},
         {"geometry", {{"type", "sphere"}, {"radiusMeters", 0.3}}}});
  EXPECT_EQ(512u, parseManifest(d.dump()).bodies.size());
  d["bodies"].push_back(d["bodies"][0]);
  refused(d, "body roster exceeds 512 simulated members");
}
