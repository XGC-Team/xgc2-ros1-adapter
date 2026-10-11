#include "xgc_scene/lidar/engine.hpp"

#include <gtest/gtest.h>

#include <cstring>
#include <thread>

using namespace xgc_scene::lidar;
using namespace xgc2_math::scene;

namespace {
struct ReferenceSink : Sink {
  struct Delivery {
    std::vector<std::uint8_t> bytes;
    std::uint64_t revision;
    std::int64_t stamp;
    ReferenceAction action;
  };
  std::vector<Delivery> delivered;
  std::thread::id reference_thread;
  std::size_t scans = 0;
  void onPoints(std::size_t, CloudView) override { ++scans; }
  void onBeams(std::size_t, CloudView) override { FAIL(); }
  void onReference(CloudView c, ReferenceAction action) override {
    std::vector<std::uint8_t> bytes;
    if (c.count)
      bytes.assign(c.data, c.data + c.byteSize());
    delivered.push_back({std::move(bytes), c.revision, c.stamp_ns, action});
    reference_thread = std::this_thread::get_id();
    EXPECT_EQ(12u, c.point_step);
  }
};
FleetManifest referenceOnly() {
  return parseManifest(
      R"({"schemaVersion":1,"robots":[],"referenceCloud":{"surfaceSpacing":0.2}})");
}
} // namespace

TEST(Reference, ExteriorFullSceneOncePerRevisionWithDemand) {
  ReferenceSink sink;
  Engine engine(referenceOnly(), sink);
  EXPECT_EQ(1u, engine.threads());
  const std::vector<Obstacle> obstacles{Obstacle::box({3, 0, 1}, {2, 2, 2}),
                                        Obstacle::sphere({3, 0, 1}, 0.5),
                                        Obstacle::sphere({100, 0, 1}, 1)};
  engine.setScene(obstacles, 1);
  EXPECT_EQ(0u, engine.tick(100));
  ASSERT_EQ(1u, sink.delivered.size());
  EXPECT_EQ(ReferenceAction::Clear, sink.delivered.back().action);
  engine.setReferenceDemand(true);
  engine.tick(200);
  ASSERT_EQ(2u, sink.delivered.size());
  const LidarScene expected(obstacles, 0.2, false);
  std::vector<std::uint8_t> bytes(12 * expected.sampleCount());
  for (std::size_t i = 0; i < expected.sampleCount(); ++i) {
    const auto &p = expected.samples()[i];
    const float xyz[] = {static_cast<float>(p.x()), static_cast<float>(p.y()),
                         static_cast<float>(p.z())};
    std::memcpy(bytes.data() + 12 * i, xyz, sizeof xyz);
  }
  EXPECT_EQ(bytes, sink.delivered.back().bytes);
  EXPECT_EQ(200, sink.delivered.back().stamp);
  EXPECT_EQ(ReferenceAction::Publish, sink.delivered.back().action);
  EXPECT_EQ(std::this_thread::get_id(), sink.reference_thread);
  EXPECT_EQ(0u, sink.scans);
  engine.tick(300);
  engine.setReferenceDemand(false);
  engine.setReferenceDemand(true);
  engine.tick(400);
  EXPECT_EQ(2u, sink.delivered.size());
  EXPECT_FALSE(engine.setScene({}, 1));
  engine.tick(500);
  EXPECT_EQ(2u, sink.delivered.size());
  engine.setScene({}, 2);
  engine.tick(600);
  ASSERT_EQ(4u, sink.delivered.size());
  EXPECT_EQ(ReferenceAction::Clear, sink.delivered[2].action);
  EXPECT_EQ(ReferenceAction::Publish, sink.delivered[3].action);
  EXPECT_TRUE(sink.delivered.back().bytes.empty());
  EXPECT_EQ(2u, sink.delivered.back().revision);
}

TEST(Reference, DisabledReplacementAndWaitingTruthClearOnEnable) {
  ReferenceSink sink;
  Engine engine(referenceOnly(), sink);
  engine.setReferenceDemand(true);
  engine.setScene({Obstacle::sphere({3, 0, 1}, 1)}, 1);
  engine.tick(100);
  ASSERT_FALSE(sink.delivered.back().bytes.empty());
  engine.setEnabled(false);
  engine.setScene({}, 2);
  engine.tick(200);
  EXPECT_EQ(2u, sink.delivered.size());
  engine.setEnabled(true);
  engine.tick(300);
  ASSERT_EQ(4u, sink.delivered.size());
  EXPECT_TRUE(sink.delivered[2].bytes.empty());
  EXPECT_TRUE(sink.delivered[3].bytes.empty());
  engine.setScene({Obstacle::sphere({4, 0, 1}, 1)}, 3);
  engine.tick(400);
  ASSERT_FALSE(sink.delivered.back().bytes.empty());
  engine.setEnabled(false);
  engine.invalidateScene(4);
  engine.tick(500);
  EXPECT_EQ(6u, sink.delivered.size());
  engine.setReferenceDemand(false);
  engine.setEnabled(true);
  engine.tick(600);
  ASSERT_EQ(7u, sink.delivered.size());
  EXPECT_EQ(ReferenceAction::Clear, sink.delivered.back().action);
  EXPECT_EQ(4u, sink.delivered.back().revision);
  EXPECT_FALSE(engine.sceneReady());
  engine.tick(700);
  EXPECT_EQ(7u, sink.delivered.size());
  EXPECT_THROW(engine.setScene({Obstacle::sphere({3, 0, 1}, -1)}, 4),
               std::invalid_argument);
  engine.tick(800);
  ASSERT_EQ(8u, sink.delivered.size());
  EXPECT_EQ(ReferenceAction::Clear, sink.delivered.back().action);
  engine.setScene({Obstacle::sphere({5, 0, 1}, 1)}, 4);
  engine.tick(900);
  EXPECT_EQ(9u, sink.delivered.size()); // clear even with no reference demand
  engine.setReferenceDemand(true);
  engine.tick(1000);
  ASSERT_EQ(10u, sink.delivered.size());
  EXPECT_EQ(ReferenceAction::Publish, sink.delivered.back().action);
  EXPECT_FALSE(sink.delivered.back().bytes.empty());
}

TEST(Reference, LocalSamplingRangeAndPolicyRemainIndependent) {
  auto manifest = parseManifest(R"({"schemaVersion":1,"robots":[
    {"namespace":"/A","mode":"penetrating","rangeMeters":2,
     "hFovDeg":360,"vFovDeg":180,"surfaceSpacing":0.2,"keepBuried":true},
    {"namespace":"/B","mode":"penetrating","surfaceSpacing":0.2}],
    "referenceCloud":{"surfaceSpacing":0.2}})");
  ReferenceSink sink;
  Engine engine(manifest, sink);
  engine.setScene({Obstacle::sphere({10, 0, 0}, 1)}, 1);
  EXPECT_NE(engine.sensorScene(0), engine.sensorScene(1));
  EXPECT_TRUE(engine.sensorScene(0)->keepBuried());
  EXPECT_FALSE(engine.sensorScene(1)->keepBuried());
  engine.setDemand(0, true);
  engine.setPose(0, xgc2_math::Pose3{}, 0);
  engine.setReferenceDemand(true);
  EXPECT_EQ(1u, engine.tick(0));
  EXPECT_EQ(1u, sink.scans);
  EXPECT_FALSE(sink.delivered.back().bytes.empty());
  // The local output is deliberately empty despite the full reference cloud.
  EXPECT_EQ(2.0, engine.manifest().robots[0].config.range);
}
