#include "xgc_scene/lidar/engine.hpp"

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>
#include <xgc2_math/scene/observation_contract.hpp>

#include <cmath>
#include <cstring>
#include <limits>

using namespace xgc_scene::lidar;
using namespace xgc2_math::scene;
using xgc2_math::Pose3;
using Json = nlohmann::json;

namespace {
constexpr std::int64_t second = 1000000000;
struct Record {
  std::vector<std::uint8_t> bytes;
  std::int64_t stamp;
  std::size_t count;
  std::uint32_t step;
  std::uint64_t revision;
};
struct Capture : Sink {
  explicit Capture(std::size_t n) : points(n), beams(n) {}
  std::vector<std::vector<Record>> points, beams;
  void onPoints(std::size_t i, CloudView c) override {
    points[i].push_back({{c.data, c.data + c.byteSize()},
                         c.stamp_ns,
                         c.count,
                         c.point_step,
                         c.revision});
  }
  void onBeams(std::size_t i, CloudView c) override {
    beams[i].push_back({{c.data, c.data + c.byteSize()},
                        c.stamp_ns,
                        c.count,
                        c.point_step,
                        c.revision});
  }
  void onReference(CloudView, ReferenceAction) override {}
};
FleetManifest fleet(std::size_t n = 1, const std::string &mode = "raycast",
                    bool beams = false) {
  Json doc{{"schemaVersion", 1}, {"robots", Json::array()}};
  for (std::size_t i = 0; i < n; ++i)
    doc["robots"].push_back({{"namespace", "/robot" + std::to_string(i)},
                             {"mode", mode},
                             {"hFovDeg", 90},
                             {"vFovDeg", 30},
                             {"hRes", 9},
                             {"vRes", 3},
                             {"rangeMeters", 8},
                             {"surfaceSpacing", 0.2},
                             {"publishBeams", beams}});
  return parseManifest(doc.dump());
}
Pose3 pose(double x = 0, double y = 0, double z = 1) {
  Pose3 p;
  p.position = {x, y, z};
  return p;
}
FleetManifest roster(const std::string &mode = "raycast",
                     bool explicit_pose = false) {
  Json robot{{"namespace", "/A"}, {"mode", mode},    {"bodyId", 41},
             {"hFovDeg", 90},     {"vFovDeg", 0},    {"hRes", 1},
             {"vRes", 1},         {"rangeMeters", 8}};
  if (explicit_pose)
    robot["poseTopic"] = "/sensor/A";
  return parseManifest(
      Json{{"schemaVersion", 1},
           {"robots", Json::array({robot})},
           {"bodies",
            Json::array(
                {{{"id", 41},
                  {"namespace", "/A"},
                  {"poseTopic", "/truth/A"},
                  {"geometry", {{"type", "sphere"}, {"radiusMeters", 0.4}}}},
                 {{"id", 7},
                  {"namespace", "/B"},
                  {"poseTopic", "/truth/B"},
                  {"geometry", {{"type", "sphere"}, {"radiusMeters", 0.4}}}}})}}
          .dump());
}
float firstX(const Record &record) {
  float x;
  std::memcpy(&x, record.bytes.data(), sizeof x);
  return x;
}
std::vector<std::uint8_t> packedBeams(const BeamScan &scan) {
  std::vector<std::uint8_t> bytes(32 * scan.size());
  for (std::size_t i = 0; i < scan.size(); ++i) {
    const auto &d = scan.direction[i];
    const float record[8] = {static_cast<float>(scan.origin.x()),
                             static_cast<float>(scan.origin.y()),
                             static_cast<float>(scan.origin.z()),
                             static_cast<float>(d.x()),
                             static_cast<float>(d.y()),
                             static_cast<float>(d.z()),
                             static_cast<float>(scan.range[i]),
                             static_cast<float>(scan.hit[i])};
    std::memcpy(bytes.data() + 32 * i, record, sizeof record);
  }
  return bytes;
}
} // namespace

TEST(Engine, SharedScenesPatternsAndSerialScanIntoByteEquality) {
  const auto default_threads =
      defaultScanThreads(5, std::thread::hardware_concurrency());
  for (const auto mode : {"raycast", "penetrating", "depth_frustum"})
    for (const bool publish_beams : {false, true})
      for (const auto threads :
           {std::size_t{1}, std::size_t{2}, default_threads}) {
        auto m = fleet(5, mode, publish_beams);
        for (auto &s : m.robots) {
          s.config.noise_std = 0.01;
          s.config.seed = 42;
        }
        EngineOptions options;
        options.worker_threads = static_cast<int>(threads);
        options.vehicle_bodies = true;
        Capture sink(5);
        Engine engine(m, sink, options);
        engine.setScene({Obstacle::box({4, 0, 1}, {0.4, 6, 2}),
                         Obstacle::sphere({-3, -3, 1}, 0.8),
                         Obstacle::cylinder({0, 4, 1.5}, 0.5, 3)},
                        1);
        std::vector<std::optional<WorldLidar>> serial(5), beam_serial(5);
        for (std::size_t i = 0; i < 5; ++i) {
          EXPECT_EQ(engine.sensorScene(0), engine.sensorScene(i));
          EXPECT_EQ(engine.beamPattern(0), engine.beamPattern(i));
          serial[i].emplace(m.robots[i].config);
          beam_serial[i].emplace(m.robots[i].config);
          serial[i]->setScene(engine.sensorScene(i));
          beam_serial[i]->setScene(engine.sensorScene(i));
          engine.setDemand(i, true);
        }
        for (int tick = 0; tick < 6; ++tick) {
          const auto now = second + tick * (second / 10);
          std::vector<VehicleBody> bodies;
          std::vector<Pose3> poses;
          for (std::size_t i = 0; i < 5; ++i) {
            auto p = pose(0.1 * tick, 0.4 * i);
            p.orientation = Eigen::Quaterniond(
                Eigen::AngleAxisd(0.12 * i, Eigen::Vector3d::UnitZ()));
            poses.push_back(p);
            engine.setPose(i, p, now - 1234567);
            bodies.push_back({m.robots[i].body_id, p.position, 0.3});
          }
          ASSERT_EQ(5u, engine.tick(now));
          for (std::size_t i = 0; i < 5; ++i) {
            std::vector<VehicleBody> others;
            for (const auto &b : bodies)
              if (b.id != m.robots[i].body_id)
                others.push_back(b);
            auto p = sensorWorldPose(poses[i], Pose3{});
            const auto q = p.orientation.normalized();
            std::vector<std::uint8_t> bytes;
            const auto count =
                serial[i]->scanInto(p.position, q, others, false, &bytes);
            const auto &delivered = sink.points[i].back();
            EXPECT_EQ(bytes, delivered.bytes) << mode << " threads=" << threads;
            EXPECT_EQ(count, delivered.count);
            EXPECT_EQ(now - 1234567, delivered.stamp);
            EXPECT_EQ(12u, delivered.step);
            if (publish_beams &&
                m.robots[i].config.mode != SensorConfig::kPenetrating) {
              BeamScan beams;
              beam_serial[i]->scanBeamsInto(p.position, q, others, &beams);
              EXPECT_EQ(packedBeams(beams), sink.beams[i].back().bytes);
              EXPECT_EQ(now - 1234567, sink.beams[i].back().stamp);
              EXPECT_EQ(32u, sink.beams[i].back().step);
            } else
              EXPECT_TRUE(sink.beams[i].empty());
          }
        }
      }
}

TEST(Engine, ExactCadenceOverTenSimulationSeconds) {
  auto m = fleet(8);
  const double rates[] = {0.5, 1, 3, 7, 10, 17, 29.97, 100};
  for (std::size_t i = 0; i < m.robots.size(); ++i) {
    m.robots[i].rate_hz = rates[i];
    m.robots[i].config.h_res = 1;
    m.robots[i].config.v_res = 1;
  }
  Capture sink(8);
  Engine engine(m, sink);
  engine.setScene({}, 1);
  for (std::size_t i = 0; i < 8; ++i)
    engine.setDemand(i, true);
  for (std::int64_t elapsed = 0; elapsed < 10 * second; elapsed += 1000000) {
    const auto now = second + elapsed;
    for (std::size_t i = 0; i < 8; ++i)
      engine.setPose(i, pose(), now);
    engine.tick(now);
  }
  for (std::size_t i = 0; i < 8; ++i) {
    ASSERT_EQ(static_cast<std::size_t>(std::ceil(10 * rates[i])),
              sink.points[i].size())
        << rates[i];
    for (std::size_t j = 0; j < sink.points[i].size(); ++j) {
      const long double ideal = second + j * (1000000000.0L / rates[i]);
      EXPECT_GE(static_cast<long double>(sink.points[i][j].stamp), ideal);
      EXPECT_LT(static_cast<long double>(sink.points[i][j].stamp),
                ideal + 1000000.0L);
    }
  }
}

TEST(Engine, UndemandedAndDisabledScansDoNotConsumeSeededNoise) {
  for (const auto mode : {"penetrating", "raycast", "depth_frustum"}) {
    auto m = fleet(2, mode, true);
    for (auto &s : m.robots) {
      s.config.noise_std = 0.01;
      s.config.seed = 42;
    }
    Capture serial_sink(2), pool_sink(2);
    EngineOptions options;
    options.worker_threads = 1;
    options.pose_timeout = 30;
    Engine serial(m, serial_sink, options);
    options.worker_threads = 2;
    Engine pooled(m, pool_sink, options);
    for (auto *engine : {&serial, &pooled}) {
      engine->setScene({Obstacle::box({4, 0, 1}, {1, 4, 2})}, 1);
      for (std::size_t i = 0; i < 2; ++i)
        engine->setPose(i, pose(), second - 7654321);
    }
    for (std::size_t i = 0; i < 2; ++i)
      serial.setDemand(i, true);
    for (int t = 0; t < 4; ++t) {
      serial.tick(second + t * second / 10);
      EXPECT_EQ(0u, pooled.tick(second + t * second / 10));
    }
    for (std::size_t i = 0; i < 2; ++i)
      pooled.setDemand(i, true);
    for (int t = 0; t < 4; ++t)
      pooled.tick(2 * second + t * second / 10);
    for (std::size_t i = 0; i < 2; ++i)
      for (std::size_t t = 0; t < 4; ++t) {
        EXPECT_EQ(serial_sink.points[i][t].bytes, pool_sink.points[i][t].bytes);
        EXPECT_EQ(second - 7654321, pool_sink.points[i][t].stamp);
      }
    pooled.setEnabled(false);
    EXPECT_EQ(0u, pooled.tick(3 * second));
    pooled.setEnabled(true);
    pooled.setEnabled(0, false);
    EXPECT_EQ(1u, pooled.tick(3 * second));
    EXPECT_EQ(4u, pool_sink.points[0].size());
    pooled.setEnabled(0, true);
    pooled.setDemand(1, false);
    EXPECT_EQ(1u, pooled.tick(4 * second));
    EXPECT_EQ(serial_sink.points[0].size() + 1, pool_sink.points[0].size());
  }
}

TEST(Engine, PoseValidationExactStampsAgeAndClockReset) {
  Capture sink(1);
  Engine engine(fleet(), sink);
  engine.setScene({}, 1);
  engine.setDemand(0, true);
  EXPECT_EQ(0u, engine.tick(second));
  EXPECT_TRUE(engine.setPose(0, pose(), second - 123));
  EXPECT_EQ(1u, engine.tick(second));
  EXPECT_EQ(second - 123, sink.points[0].back().stamp);
  auto invalid = pose();
  invalid.orientation.coeffs().setZero();
  EXPECT_FALSE(engine.setPose(0, invalid, second));
  EXPECT_EQ(0u, engine.tick(2 * second));
  invalid = pose();
  invalid.position.x() = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(engine.setPose(0, invalid, 2 * second));
  EXPECT_EQ(0u, engine.tick(2 * second));
  EXPECT_FALSE(engine.setPose(0, pose(), 2 * second, "map"));
  EXPECT_EQ(0u, engine.tick(2 * second));
  engine.setPose(0, pose(), 3 * second);
  EXPECT_EQ(0u, engine.tick(2 * second));
  engine.setPose(0, pose(), 0);
  EXPECT_EQ(0u, engine.tick(second));
  EXPECT_EQ(1u, engine.tick(0)); // Math permits zero simulation time.
  EXPECT_EQ(0, sink.points[0].back().stamp);
  engine.setPose(0, pose(), -1);
  EXPECT_EQ(0u, engine.tick(0));
  engine.setPose(0, pose(), second);
  EXPECT_EQ(1u, engine.tick(second));
  EXPECT_EQ(1u, engine.tick(second + second / 2));
  EXPECT_EQ(0u, engine.tick(second + second / 2 + second / 10));
  engine.setPose(0, pose(), 10 * second);
  EXPECT_EQ(1u, engine.tick(10 * second));
  EXPECT_EQ(0u, engine.tick(10 * second)); // no backlog/burst
  engine.setPose(0, pose(), 0);
  EXPECT_EQ(1u, engine.tick(0));
}

TEST(Engine, UnsensedRosterBodiesSelfExclusionWallOcclusionAndFailClosed) {
  Capture sink(1);
  Engine engine(roster(), sink);
  engine.setScene({}, 1);
  engine.setDemand(0, true);
  engine.setBodyPose(0, pose(), second);
  EXPECT_EQ(0u, engine.tick(second));
  engine.setBodyPose(1, pose(3), second);
  ASSERT_EQ(1u, engine.tick(second));
  ASSERT_EQ(1u, sink.points[0].back().count);
  EXPECT_FLOAT_EQ(2.6f, firstX(sink.points[0].back()));
  EXPECT_EQ(12u, sink.points[0].back().step);
  engine.setScene({Obstacle::box({1.5, 0, 1}, {0.2, 4, 3})}, 2);
  ASSERT_EQ(1u, engine.tick(second + second / 10));
  EXPECT_FLOAT_EQ(1.4f, firstX(sink.points[0].back()));
  EXPECT_EQ(2u, sink.points[0].back().revision);
  engine.setScene({}, 3);
  ASSERT_EQ(1u, engine.tick(second + 2 * second / 10));
  EXPECT_FLOAT_EQ(2.6f, firstX(sink.points[0].back()));
  engine.setBodyPose(0, pose(), 2 * second);
  EXPECT_EQ(0u, engine.tick(2 * second)); // B truth stopped, roster remains.
  engine.setBodyPose(1, pose(3), 2 * second + 1);
  EXPECT_EQ(0u, engine.tick(2 * second)); // future sample fails closed.
  auto invalid = pose(3);
  invalid.orientation.coeffs().setZero();
  engine.setBodyPose(1, invalid, 2 * second);
  EXPECT_EQ(0u, engine.tick(2 * second));
  engine.setBodyPose(1, pose(3), 2 * second);
  ASSERT_EQ(1u, engine.tick(2 * second));
  EXPECT_FLOAT_EQ(2.6f, firstX(sink.points[0].back()));
  engine.setDemand(0, false);
  EXPECT_EQ(0u, engine.tick(2 * second + second / 10));
  engine.setDemand(0, true);
  ASSERT_EQ(1u, engine.tick(2 * second + 2 * second / 10));
  EXPECT_FLOAT_EQ(2.6f, firstX(sink.points[0].back()));
}

TEST(Engine, ExplicitPoseAndBodyFreePenetratingBaseline) {
  Capture sink(1);
  Engine engine(roster("raycast", true), sink);
  engine.setScene({}, 1);
  engine.setDemand(0, true);
  engine.setBodyPose(0, pose(), second);
  engine.setBodyPose(1, pose(3), second);
  EXPECT_EQ(0u,
            engine.tick(second)); // explicit sensor topic needs its own pose.
  engine.setPose(0, pose(1), second - 1234567);
  ASSERT_EQ(1u, engine.tick(second));
  EXPECT_FLOAT_EQ(
      2.6f, firstX(sink.points[0].back())); // world point, not sensor-relative
  EXPECT_EQ(second - 1234567, sink.points[0].back().stamp);
  Capture crop_sink(1);
  Engine crop(roster("penetrating"), crop_sink);
  crop.setScene({}, 1);
  crop.setDemand(0, true);
  crop.setPose(0, pose(), second);
  EXPECT_EQ(
      1u,
      crop.tick(second)); // missing roster truth does not change crop baseline.
  EXPECT_EQ(0u, crop_sink.points[0].back().count);
  EngineOptions options;
  options.vehicle_bodies = true;
  Capture opted_sink(1);
  auto opted_manifest = roster("penetrating");
  opted_manifest.robots[0].config.v_fov_deg = 180;
  Engine opted(opted_manifest, opted_sink, options);
  opted.setScene({}, 1);
  opted.setDemand(0, true);
  opted.setPose(0, pose(), second);
  EXPECT_EQ(0u, opted.tick(second));
  opted.setBodyPose(0, pose(), second);
  opted.setBodyPose(1, pose(3), second);
  EXPECT_EQ(1u, opted.tick(second));
  EXPECT_GT(opted_sink.points[0].back().count, 0u);
}

TEST(Engine, LegacyUnsensedNeighborsAndSceneReplacementMidRun) {
  Capture sink(3);
  EngineOptions options;
  options.vehicle_bodies = true;
  Engine engine(fleet(3, "penetrating"), sink, options);
  engine.setScene({}, 1);
  engine.setDemand(0, true);
  engine.setPose(0, pose(), second);
  engine.setPose(1, pose(2), second);
  engine.setPose(2, pose(0, 2), second);
  engine.setEnabled(2, false);
  ASSERT_EQ(1u, engine.tick(second));
  EXPECT_GT(sink.points[0].back().count, 0u);
  EXPECT_TRUE(sink.points[1].empty());
  EXPECT_TRUE(sink.points[2].empty());
  engine.invalidateScene(2);
  EXPECT_EQ(0u, engine.tick(second + second / 10));
  EXPECT_THROW(engine.setScene({Obstacle::sphere({2, 0, 1}, -1)}, 2),
               std::invalid_argument);
  EXPECT_EQ(0u, engine.tick(second + second / 10));
  engine.setScene({}, 2);
  EXPECT_EQ(1u, engine.tick(second + 2 * second / 10));
  engine.setScene({Obstacle::sphere({5, 0, 1}, 0.5)}, 3);
  EXPECT_EQ(1u, engine.tick(second + 3 * second / 10));
  EXPECT_GT(sink.points[0].back().count, sink.points[0].front().count);
}

TEST(Engine, NanosecondFreshnessAtLargeAbsoluteSimulationTime) {
  constexpr std::int64_t now = 1000000000000000000;
  Capture sink(1);
  Engine engine(roster(), sink);
  engine.setScene({}, 1);
  engine.setDemand(0, true);
  engine.setBodyPose(0, pose(), now);
  engine.setBodyPose(1, pose(3), now + 1);
  EXPECT_EQ(0u, engine.tick(now));
  engine.setBodyPose(1, pose(3), now - second / 2 - 1);
  EXPECT_EQ(0u, engine.tick(now));
  engine.setBodyPose(1, pose(3), now - second / 2);
  ASSERT_EQ(1u, engine.tick(now));
  EXPECT_EQ(now, sink.points[0].back().stamp);
}

TEST(Engine, ThreadOverrideIsBoundedAndInvalidOptionsAreRefused) {
  Capture sink(2);
  EngineOptions options;
  options.worker_threads = 100;
  Engine bounded(fleet(2), sink, options);
  EXPECT_EQ(2u, bounded.threads());
  options.pose_timeout = 0;
  EXPECT_THROW(Engine(fleet(2), sink, options), std::invalid_argument);
  options.pose_timeout = 0.5;
  options.vehicle_bodies = true;
  options.vehicle_body_radius = 0;
  EXPECT_THROW(Engine(fleet(2), sink, options), std::invalid_argument);
}
