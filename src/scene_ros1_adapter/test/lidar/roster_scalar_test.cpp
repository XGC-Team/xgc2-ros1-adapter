#include "xgc_scene/lidar/roster.hpp"
#include <gtest/gtest.h>
#include <limits>

using namespace xgc_scene;
using namespace xgc_scene::lidar;

namespace {
SessionInput input(Json lidar) {
  return parseSessionInput(Json{{"runMode", "simulation"}, {"scene", {{"simulator", "xsim"}}},
      {"robots", Json::array({{{"namespace", "/robot"}, {"binding", {
          {"hybridSource", "simulation"}, {"simulationSensors", {{"simpleLidar", std::move(lidar)}}}}}}})},
      {"visualizationTopics", Json::array()}}.dump());
}
Json selection() { return {{"enabled", true}, {"mode", "raycast"}, {"acceleration", "cpu"}}; }
void rejected(const SessionInput &session, const char *field, bool point_cloud) {
  try {
    buildFleet(session, {}, point_cloud);
    FAIL() << "accepted invalid authored " << field;
  } catch (const SceneError &error) {
    EXPECT_NE(std::string::npos, std::string(error.what()).find(std::string("simpleLidar.") + field));
  }
}
} // namespace

TEST(RosterScalars, InvalidNumbersCannotBecomeDefaultsOrDisappearInProducerFiltering) {
  for (const auto &bound : {std::pair<const char *, int>{"rateHz", 100}, {"rangeMeters", 200},
                           {"hFovDeg", 360}, {"vFovDeg", 180}, {"hRes", 4096}, {"vRes", 4096}}) {
    for (const auto &value : Json::array({-1, bound.second + 1, nullptr, false, true, "2", Json::array(), Json::object()})) {
      for (const bool enabled : {false, true}) {
        for (const bool point_cloud : {false, true}) {
          SCOPED_TRACE(Json({{"field", bound.first}, {"value", value}, {"enabled", enabled}, {"pointCloud", point_cloud}}).dump());
          auto authored = selection(); authored["enabled"] = enabled; authored[bound.first] = value;
          rejected(input(authored), bound.first, point_cloud);
        }
      }
    }
  }
  for (const auto *field : {"hRes", "vRes"}) {
    for (const double fraction : {2.0, 2.5}) {
      auto authored = selection(); authored[field] = fraction;
      rejected(input(authored), field, true);
    }
  }
}

TEST(RosterScalars, ValidationPrecedesPhysicalAndHybridRosterFiltering) {
  auto authored = selection(); authored["hRes"] = 2.5;
  for (const auto *run_mode : {"physical", "hybrid"}) {
    auto session = input(authored); session.run_mode = run_mode;
    session.robots[0]["binding"]["hybridSource"] = "physical";
    rejected(session, "hRes", true);
  }
}

TEST(RosterScalars, NonfiniteNumbersAreRejectedBeforeSerialization) {
  for (const auto *field : {"rateHz", "rangeMeters", "hFovDeg", "vFovDeg"}) {
    for (const double value : {std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()}) {
      auto session = input(selection());
      session.robots[0]["binding"]["simulationSensors"]["simpleLidar"][field] = value;
      rejected(session, field, true);
    }
  }
}

TEST(RosterScalars, SingleSampleResolutionRequiresExplicitPenetratingMode) {
  for (const auto *field : {"hRes", "vRes"}) {
    for (const auto *mode : {"", "raycast", "depth_frustum", "penetrating"}) {
      for (const bool enabled : {false, true}) {
        auto authored = selection(); authored[field] = 1; authored["mode"] = mode; authored["enabled"] = enabled;
        if (std::string(mode) == "penetrating")
          EXPECT_NO_THROW(buildFleet(input(authored), {}, true));
        else
          rejected(input(authored), field, true);
      }
    }
  }
}

TEST(RosterScalars, ZeroDefaultsBoundsAndBooleanSelectionsKeepTheirContract) {
  auto authored = selection();
  for (const auto *field : {"rateHz", "rangeMeters", "hFovDeg", "vFovDeg", "hRes", "vRes"}) authored[field] = 0;
  auto fleet = buildFleet(input(authored), {}, true);
  ASSERT_EQ(1u, fleet.robots.size());
  EXPECT_EQ(10.0, fleet.robots[0].rate_hz);
  EXPECT_EQ(20.0, fleet.robots[0].config.range);
  EXPECT_EQ(360, fleet.robots[0].config.h_res);
  EXPECT_EQ(32, fleet.robots[0].config.v_res);
  authored.update({{"rateHz", 100}, {"rangeMeters", 200}, {"hFovDeg", 360}, {"vFovDeg", 180}, {"hRes", 4096}, {"vRes", 4096}});
  EXPECT_NO_THROW(buildFleet(input(authored), {}, true));
  EXPECT_EQ(1u, buildFleet(input(true), {}, true).robots.size());
  EXPECT_TRUE(buildFleet(input(false), {}, true).robots.empty());
}

TEST(RosterScalars, DisabledSelectionsStillRequireTypedModePresetBackendAndBooleans) {
  for (const auto *field : {"mode", "preset", "acceleration", "enabled", "bodyReturns", "publishBeams"}) {
    auto authored = selection(); authored["enabled"] = false; authored[field] = nullptr;
    rejected(input(authored), field, false);
  }
}
