#include "xgc_scene/lidar/engine.hpp"
#include <atomic>
#include <cassert>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

using namespace xgc_scene::lidar;
using namespace xgc2_math::scene;

namespace {
constexpr std::int64_t kBaseStamp = 1000000000LL;
constexpr std::int64_t kStepStamp = 200000000LL;

struct Probe : Sink {
  bool through = false;
  std::atomic<std::size_t> deliveries{0};
  std::vector<std::vector<std::vector<std::uint8_t>>> clouds;
  std::vector<std::vector<std::int64_t>> stamps;
  explicit Probe(std::size_t robots) : clouds(robots), stamps(robots) {}
  void onPoints(std::size_t i, CloudView c) override {
    assert(c.count > 0 && c.point_step == 12);
    std::size_t behind = 0;
    for (std::size_t n = 0; n < c.count; ++n) {
      float x;
      std::memcpy(&x, c.data + n * 12, 4);
      if (x > 7) ++behind;
    }
    assert((behind > 0) == through);
    clouds[i].emplace_back(c.data, c.data + c.byteSize());
    stamps[i].push_back(c.stamp_ns);
    ++deliveries;
  }
  void onBeams(std::size_t, CloudView) override { assert(false); }
  void onReference(CloudView, ReferenceAction) override {}
};

struct Run {
  std::unique_ptr<Probe> probe;
  std::string error;
  bool completed = false;
};

Run run(const std::string &backend, bool through, int robots, int rounds,
        const std::vector<Obstacle> &scene, bool fixed) {
  Run result;
  result.probe = std::make_unique<Probe>(robots);
  Probe &sink = *result.probe;
  sink.through = through;
  try {
    FleetManifest fleet;
    for (int n = 0; n < robots; ++n) {
      SensorSpec sensor;
      sensor.name_space = "/uav" + std::to_string(n + 1);
      sensor.backend = backend;
      sensor.config.mode = through ? SensorConfig::kPenetrating : SensorConfig::kRaycast;
      sensor.config.range = 20;
      sensor.config.min_range = .2;
      sensor.config.h_fov_deg = sensor.config.v_fov_deg = 60;
      sensor.config.h_res = sensor.config.v_res = 120;
      sensor.config.surface_spacing = n == robots - 1 ? .2 : .1;
      fleet.robots.push_back(sensor);
    }
    Engine engine(std::move(fleet), sink);
    if (fixed) {
      LidarScene sampled(scene, .1, true);
      std::vector<std::uint8_t> bytes(sampled.sampleCount() * 12);
      for (std::size_t i = 0; i < sampled.sampleCount(); ++i) {
        float xyz[3];
        for (int a = 0; a < 3; ++a) xyz[a] = sampled.samples()[i][a];
        std::memcpy(bytes.data() + i * 12, xyz, 12);
      }
      engine.setPointCloud(bytes, 1);
    } else {
      engine.setScene(scene, 1);
    }
    for (int round = 0; round < rounds; ++round) {
      const std::int64_t stamp = kBaseStamp + round * kStepStamp;
      for (int n = 0; n < robots; ++n) {
        xgc2_math::Pose3 pose;
        pose.position.x() = round >= 2 ? .01 * round : 0;
        engine.setPose(n, pose, stamp);
        engine.setDemand(n, true);
      }
      assert(engine.tick(stamp) == static_cast<std::size_t>(robots));
    }
    result.completed = true;
  } catch (const std::exception &error) {
    result.error = error.what();
  }
  return result;
}

bool same_output(const Probe &a, const Probe &b) {
  if (a.deliveries.load() != b.deliveries.load() || a.clouds.size() != b.clouds.size()) return false;
  for (std::size_t i = 0; i < a.clouds.size(); ++i)
    if (a.clouds[i] != b.clouds[i] || a.stamps[i] != b.stamps[i]) return false;
  return true;
}

void verify_stamps(const Probe &probe, int robots, int rounds) {
  for (int n = 0; n < robots; ++n) {
    assert(probe.stamps[n].size() == static_cast<std::size_t>(rounds));
    for (int round = 0; round < rounds; ++round)
      assert(probe.stamps[n][round] == kBaseStamp + round * kStepStamp);
  }
}
}  // namespace

int main(int argc, char **argv) {
  const int robots = argc > 2 ? std::stoi(argv[2]) : 20;
  const int rounds = argc > 2 ? 12 : 3;
  const bool fixed = argc > 3 && std::string(argv[3]) == "fixed";
  std::vector<Obstacle> scene = {Obstacle::box({5, 0, 0}, {1, 8, 8}),
                                 Obstacle::box({9, 0, 0}, {1, 2, 2})};
  if (argc > 2)
    for (int i = 0; i < 100; ++i)
      scene.push_back(Obstacle::box({15. + (i / 10) * 3., (i % 10 - 5) * 3., 0}, {1, 2, 3}));

  for (const std::string backend : {"cpu", "gpu", "auto"})
    for (bool through : {false, true}) {
      if (argc >= 2 && backend != argv[1]) continue;
      if (backend == "gpu") {
        const Run gpu = run(backend, through, robots, rounds, scene, fixed);
        if (gpu.completed) {
          assert(gpu.probe->deliveries.load() == static_cast<std::size_t>(robots * rounds));
          verify_stamps(*gpu.probe, robots, rounds);
          std::cout << "PASS scene adapter gpu real GPU through=" << through
                    << " native_outputs=" << gpu.probe->deliveries.load() << "\n";
        } else {
          // Explicit GPU must fail closed and must not deliver a partial cloud.
          assert(gpu.probe->deliveries.load() == 0);
          std::cout << "PASS scene adapter gpu FAIL CLOSED through=" << through
                    << " error=" << gpu.error << "\n";
        }
        continue;
      }
      const Run result = run(backend, through, robots, rounds, scene, fixed);
      assert(result.completed);
      assert(result.probe->deliveries.load() == static_cast<std::size_t>(robots * rounds));
      verify_stamps(*result.probe, robots, rounds);
      if (backend == "auto") {
        const Run cpu = run("cpu", through, robots, rounds, scene, fixed);
        assert(cpu.completed);
        assert(same_output(*result.probe, *cpu.probe));
        std::cout << "PASS scene adapter auto->cpu through=" << through
                  << " native_outputs=" << result.probe->deliveries.load()
                  << " pose_identity=stamp+world_bytes\n";
      } else {
        std::cout << "PASS scene adapter " << backend << " through=" << through
                  << " native_outputs=" << result.probe->deliveries.load()
                  << " pose_identity=stamp+world_bytes\n";
      }
    }
}
