#include "xgc_scene/lidar/engine.hpp"

#include <gtest/gtest.h>
#include <cstring>

// Link-time double at the existing static GPU-result boundary. It has no
// geometry queries: fixed map bytes let these tests verify that Engine still
// invokes the GPU path, then composes targets with the shared Math helper.
namespace xgc_scene::lidar {
namespace { unsigned static_scans = 0; }
struct GpuMap::State { std::vector<float> xyz; };
GpuMap::GpuMap(const std::vector<float>& xyz, const xgc2_math::scene::SensorConfig&)
    : state_(std::make_unique<State>()) { state_->xyz = xyz; }
GpuMap::GpuMap(const std::shared_ptr<const xgc2_math::scene::LidarScene>&,
               const xgc2_math::scene::SensorConfig&) : state_(std::make_unique<State>()) {}
GpuMap::~GpuMap() = default;
void GpuMap::scan(const xgc2_math::scene::SensorConfig&, const xgc2_math::Pose3&,
                  double, std::vector<std::uint8_t>& xyz) {
  ++static_scans;
  xyz.resize(state_->xyz.size() * sizeof(float));
  if (!xyz.empty()) std::memcpy(xyz.data(), state_->xyz.data(), xyz.size());
}
} // namespace xgc_scene::lidar

namespace {
using namespace xgc_scene::lidar;
using namespace xgc2_math::scene;
constexpr std::int64_t second = 1000000000;
struct Capture : Sink {
  std::vector<std::uint8_t> points;
  void onPoints(std::size_t, CloudView cloud) override {
    points.resize(cloud.byteSize());
    if (!points.empty()) std::memcpy(points.data(), cloud.data, points.size());
  }
  void onBeams(std::size_t, CloudView) override { ADD_FAILURE(); }
  void onReference(CloudView, ReferenceAction) override {}
};
std::vector<std::uint8_t> wall() {
  const float point[] = {4, 0, 1};
  std::vector<std::uint8_t> result(sizeof point);
  std::memcpy(result.data(), point, sizeof point);
  return result;
}
bool containsPoint(const std::vector<std::uint8_t>& bytes, const Eigen::Vector3d& expected) {
  for (std::size_t i = 0; i < bytes.size() / 12; ++i) {
    float xyz[3];
    std::memcpy(xyz, bytes.data() + i * 12, sizeof xyz);
    if ((Eigen::Vector3d(xyz[0], xyz[1], xyz[2]) - expected).norm() < 1e-5) return true;
  }
  return false;
}
FleetManifest manifest(const std::string& backend, bool penetrating, bool target) {
  FleetManifest result;
  SensorSpec scanner;
  scanner.backend = backend;
  scanner.body_id = 1;
  scanner.name_space = "/scanner";
  scanner.config.h_fov_deg = 90;
  scanner.config.v_fov_deg = 30;
  scanner.config.h_res = 8;
  scanner.config.v_res = 4;
  scanner.config.range = 8;
  scanner.config.surface_spacing = .1;
  scanner.config.mode = penetrating ? SensorConfig::kPenetrating : SensorConfig::kRaycast;
  result.robots.push_back(scanner);
  result.body_roster = true;
  result.bodies = {{2, "/target_without_sensor", "/target/pose", .4, target}};
  return result;
}

TEST(SavedCloudGpuBodies, GpuAndAutoKeepTheirStaticBackendInBothModes) {
  for (const std::string backend : {"gpu", "auto"})
    for (const bool penetrating : {false, true}) {
      SCOPED_TRACE(backend + (penetrating ? " penetrating" : " raycast"));
      Capture sink;
      Engine engine(manifest(backend, penetrating, true), sink);
      engine.setPointCloud(wall(), 1);
      engine.setDemand(0, true);
      xgc2_math::Pose3 scanner, body;
      scanner.position = {0, 0, 1};
      body.position = {3, 0, 1};
      engine.setPose(0, scanner, second);
      engine.setBodyPose(0, body, second);
      const auto before = xgc_scene::lidar::static_scans;
      ASSERT_EQ(1u, engine.tick(second));
      EXPECT_EQ(before + 1, xgc_scene::lidar::static_scans);
      ASSERT_FALSE(sink.points.empty());
      if (penetrating) {
        EXPECT_TRUE(containsPoint(sink.points, {4, 0, 1}));
        EXPECT_GT(sink.points.size(), wall().size());
      } else {
        EXPECT_TRUE(containsPoint(sink.points, {2.6, 0, 1}));
        EXPECT_FALSE(containsPoint(sink.points, {4, 0, 1}));
      }
      body.position = {6, 0, 1};
      engine.setBodyPose(0, body, second + second / 10);
      ASSERT_EQ(1u, engine.tick(second + second / 10));
      EXPECT_EQ(before + 2, xgc_scene::lidar::static_scans);
      if (!penetrating) EXPECT_TRUE(containsPoint(sink.points, {4, 0, 1}));
    }
}

TEST(SavedCloudGpuBodies, MissingOrDisabledTargetsKeepStaticResults) {
  for (const std::string backend : {"gpu", "auto"})
    for (const bool target : {false, true}) {
      Capture sink;
      Engine engine(manifest(backend, false, target), sink);
      engine.setPointCloud(wall(), 1);
      engine.setDemand(0, true);
      xgc2_math::Pose3 scanner;
      scanner.position = {0, 0, 1};
      engine.setPose(0, scanner, second);
      const auto before = xgc_scene::lidar::static_scans;
      ASSERT_EQ(1u, engine.tick(second));
      EXPECT_EQ(before + 1, xgc_scene::lidar::static_scans);
      EXPECT_EQ(wall(), sink.points);
    }
}
} // namespace
