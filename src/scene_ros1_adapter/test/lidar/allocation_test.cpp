#include "xgc_scene/lidar/engine.hpp"

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include <atomic>
#include <cstdlib>
#include <new>

namespace {
std::atomic<bool> counting{false};
std::atomic<std::size_t> allocation_count{0};
void *allocate(std::size_t size, std::size_t alignment) {
  if (counting.load(std::memory_order_relaxed))
    allocation_count.fetch_add(1, std::memory_order_relaxed);
  void *result = nullptr;
  if (alignment <= alignof(std::max_align_t))
    result = std::malloc(size ? size : 1);
  else if (posix_memalign(&result, alignment, size ? size : 1) != 0)
    result = nullptr;
  if (!result)
    throw std::bad_alloc();
  return result;
}
} // namespace

void *operator new(std::size_t n) {
  return allocate(n, alignof(std::max_align_t));
}
void *operator new[](std::size_t n) {
  return allocate(n, alignof(std::max_align_t));
}
void *operator new(std::size_t n, std::align_val_t a) {
  return allocate(n, static_cast<std::size_t>(a));
}
void *operator new[](std::size_t n, std::align_val_t a) {
  return allocate(n, static_cast<std::size_t>(a));
}
void *operator new(std::size_t n, const std::nothrow_t &) noexcept {
  try {
    return allocate(n, alignof(std::max_align_t));
  } catch (...) {
    return nullptr;
  }
}
void *operator new[](std::size_t n, const std::nothrow_t &) noexcept {
  return ::operator new(n, std::nothrow);
}
void *operator new(std::size_t n, std::align_val_t a,
                   const std::nothrow_t &) noexcept {
  try {
    return allocate(n, static_cast<std::size_t>(a));
  } catch (...) {
    return nullptr;
  }
}
void *operator new[](std::size_t n, std::align_val_t a,
                     const std::nothrow_t &) noexcept {
  return ::operator new(n, a, std::nothrow);
}
void operator delete(void *p) noexcept { std::free(p); }
void operator delete[](void *p) noexcept { std::free(p); }
void operator delete(void *p, std::size_t) noexcept { std::free(p); }
void operator delete[](void *p, std::size_t) noexcept { std::free(p); }
void operator delete(void *p, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void *p, std::align_val_t) noexcept { std::free(p); }
void operator delete(void *p, std::size_t, std::align_val_t) noexcept {
  std::free(p);
}
void operator delete[](void *p, std::size_t, std::align_val_t) noexcept {
  std::free(p);
}
void operator delete(void *p, const std::nothrow_t &) noexcept { std::free(p); }
void operator delete[](void *p, const std::nothrow_t &) noexcept {
  std::free(p);
}
void operator delete(void *p, std::align_val_t,
                     const std::nothrow_t &) noexcept {
  std::free(p);
}
void operator delete[](void *p, std::align_val_t,
                       const std::nothrow_t &) noexcept {
  std::free(p);
}

namespace {
using namespace xgc_scene::lidar;
using namespace xgc2_math::scene;
using Json = nlohmann::json;
constexpr std::int64_t second = 1000000000;
template <class Work> std::size_t allocations(Work &&work) {
  struct CountingScope {
    CountingScope() {
      allocation_count.store(0);
      counting.store(true);
    }
    ~CountingScope() { counting.store(false); }
  } scope;
  work();
  return allocation_count.load();
}
struct CountingSink : Sink {
  explicit CountingSink(std::size_t n) : points(n), beams(n) {}
  std::vector<std::size_t> points, beams;
  std::size_t references = 0;
  void onPoints(std::size_t i, CloudView) override { ++points[i]; }
  void onBeams(std::size_t i, CloudView) override { ++beams[i]; }
  void onReference(CloudView, ReferenceAction) override { ++references; }
};
FleetManifest fleet(const std::string &mode, bool beams, bool roster) {
  Json doc{{"schemaVersion", 1},
           {"robots", Json::array()},
           {"referenceCloud", {{"surfaceSpacing", 0.2}}}};
  if (roster)
    doc["bodies"] = Json::array();
  for (std::size_t i = 0; i < 6; ++i) {
    const std::string ns = "/robot" + std::to_string(i);
    doc["robots"].push_back({{"namespace", ns},
                             {"mode", mode},
                             {"hFovDeg", 90},
                             {"vFovDeg", 60},
                             {"hRes", 18},
                             {"vRes", 6},
                             {"surfaceSpacing", 0.2},
                             {"publishBeams", beams}});
    if (roster)
      doc["bodies"].push_back(
          {{"id", 10 + i},
           {"namespace", ns},
           {"poseTopic", "/truth" + ns},
           {"geometry", {{"type", "sphere"}, {"radiusMeters", 0.3}}}});
  }
  if (roster)
    doc["bodies"].push_back(
        {{"id", 91},
         {"namespace", "/unsensed"},
         {"poseTopic", "/truth/unsensed"},
         {"geometry", {{"type", "sphere"}, {"radiusMeters", 0.5}}}});
  SensorConfig config;
  config.noise_std = 0.01;
  config.seed = 42;
  return parseManifest(doc.dump(), config);
}
void poses(Engine &engine, std::int64_t now, double dx) {
  for (std::size_t i = 0; i < engine.manifest().bodies.size(); ++i) {
    xgc2_math::Pose3 p;
    p.position = {dx + 0.4 * i, 0.5, 1};
    engine.setBodyPose(i, p, now);
  }
  for (std::size_t i = 0; i < engine.manifest().robots.size(); ++i) {
    xgc2_math::Pose3 p;
    p.position = {dx + 0.4 * i, 0.0, 1};
    engine.setPose(i, p, now);
  }
}
} // namespace

TEST(Allocation, CounterSeesScalarArrayAlignedAndSeedAllocations) {
  EXPECT_EQ(1u, allocations([] {
              void *p = ::operator new(31);
              ::operator delete(p);
            }));
  EXPECT_EQ(1u, allocations([] {
              void *p = ::operator new[](31);
              ::operator delete[](p);
            }));
  EXPECT_EQ(1u, allocations([] {
              void *p = ::operator new (31, std::align_val_t{64});
              ::operator delete (p, std::align_val_t{64});
            }));
  EXPECT_GT(allocations([] {
              std::seed_seq seed{1u, 2u, 3u};
              volatile auto size = seed.size();
              (void)size;
            }),
            0u);
}

TEST(Allocation, SteadyFleetTicksIncludingNoiseBodiesBeamsAndPoseUpdates) {
  for (const auto mode : {"raycast", "penetrating", "depth_frustum"})
    for (const bool beams : {false, true})
      for (const bool roster : {false, true})
        for (const auto threads :
             {std::size_t{1}, std::size_t{2},
              defaultScanThreads(6, std::thread::hardware_concurrency())}) {
          CountingSink sink(6);
          EngineOptions options;
          options.worker_threads = static_cast<int>(threads);
          options.vehicle_bodies = true;
          Engine engine(fleet(mode, beams, roster), sink, options);
          engine.setScene({Obstacle::box({4, 0, 1}, {1, 4, 3}),
                           Obstacle::sphere({-2, 0, 1}, 1)},
                          1);
          for (std::size_t i = 0; i < 6; ++i)
            engine.setDemand(i, true);
          engine.setReferenceDemand(true);
          poses(engine, second, 0);
          ASSERT_EQ(6u, engine.tick(second)); // only warm-up scan
          const auto count = allocations([&] {
            for (int t = 1; t <= 100; ++t) {
              const auto now = second + t * second / 10;
              poses(engine, now, 0.005 * t);
              engine.tick(now);
            }
          });
          EXPECT_EQ(0u, count)
              << mode << " beams=" << beams << " roster=" << roster
              << " threads=" << threads;
          for (const auto calls : sink.points)
            EXPECT_EQ(101u, calls);
          engine.setScene({Obstacle::box({5, 0, 1}, {1, 4, 3})}, 2);
          poses(engine, 12 * second, 0);
          engine.tick(12 * second);
          EXPECT_EQ(0u, allocations([&] {
                      poses(engine, 13 * second, 0.1);
                      engine.tick(13 * second);
                    }));
        }
}

TEST(Allocation, IdleDisabledWaitingAndDeferredReferenceDelivery) {
  CountingSink sink(6);
  Engine engine(fleet("penetrating", false, true), sink);
  engine.setScene({Obstacle::sphere({4, 0, 1}, 1)}, 1);
  poses(engine, 0, 0);
  EXPECT_EQ(0u, allocations([&] {
              for (int i = 0; i < 100; ++i)
                engine.tick(i);
              engine.setReferenceDemand(true);
              engine.tick(100);
              engine.setEnabled(false);
              engine.tick(200);
            }));
  engine.invalidateScene(2);
  EXPECT_EQ(0u, allocations([&] {
              engine.tick(300);
              engine.setEnabled(true);
              engine.tick(400);
              engine.tick(500);
            }));
  for (const auto scans : sink.points)
    EXPECT_EQ(0u, scans);
  EXPECT_EQ(3u, sink.references); // first clear, publication, deferred clear
}

TEST(Allocation, SceneRevisionsPrepareOnlyDemandedSensorsLazily) {
  for (const auto mode : {"raycast", "penetrating", "depth_frustum"})
    for (const bool beams : {false, true}) {
      auto many_manifest = fleet(mode, beams, false);
      many_manifest.reference.reset();
      auto one_manifest = many_manifest;
      one_manifest.robots.resize(1);
      CountingSink one_sink(1), many_sink(6);
      EngineOptions options;
      options.worker_threads = 1;
      Engine one(one_manifest, one_sink, options);
      Engine many(many_manifest, many_sink, options);
      std::vector<Obstacle> obstacles{Obstacle::box({4, 0, 1}, {1, 4, 3})};
      for (std::uint64_t revision = 1; revision <= 3; ++revision) {
        obstacles.push_back(Obstacle::sphere({-2, 0, 1}, revision));
        const auto one_allocations =
            allocations([&] { one.setScene(obstacles, revision); });
        const auto many_allocations =
            allocations([&] { many.setScene(obstacles, revision); });
        // Compiling the same shared geometry must cost the same allocations,
        // regardless of the number of sensor output/scratch buffers.
        EXPECT_EQ(one_allocations, many_allocations)
            << mode << " beams=" << beams << " revision=" << revision;
        EXPECT_EQ(one.sensorScene(0)->sampleCount(),
                  many.sensorScene(5)->sampleCount());
        const auto now = static_cast<std::int64_t>(revision) * second;
        poses(one, now, 0);
        poses(many, now, 0);
        EXPECT_EQ(0u, allocations([&] { EXPECT_EQ(0u, many.tick(now)); }));
        one.setDemand(0, true);
        many.setDemand(0, true);
        const auto one_scan = allocations([&] { one.tick(now); });
        const auto many_scan = allocations([&] { many.tick(now); });
        EXPECT_GT(one_scan, 0u); // preparation happens when the scan needs it
        EXPECT_EQ(one_scan, many_scan);
        EXPECT_EQ(0u, allocations([&] {
                    one.tick(now + second / 10);
                    many.tick(now + second / 10);
                  }));
        EXPECT_EQ(2 * revision, one_sink.points[0]);
        EXPECT_EQ(one_sink.points[0], many_sink.points[0]);
        for (std::size_t i = 1; i < 6; ++i)
          EXPECT_EQ(0u, many_sink.points[i]);
        one.setDemand(0, false);
        many.setDemand(0, false);
      }
    }
}
