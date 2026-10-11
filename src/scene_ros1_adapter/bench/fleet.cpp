// Same Math geometry in both paths. The baseline ports the old node's hot
// fleet loop and uses its unmodified ScanPool source from commit 2940a99.
// Each run is 100 ticks at 10 Hz (10 simulated seconds), after one warm scan.
#include "allocation_counter.hpp"
#include "xgc_scene/lidar/engine.hpp"

#include <xgc2_math/scene/cluttered_environment.hpp>
#include <xgc2_math/scene/observation_contract.hpp>
#include <xgc2_world_lidar/scan_pool.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <fstream>
#include <map>
#include <numeric>
#include <random>
#include <stdexcept>

namespace {
using namespace xgc_scene::lidar;
using namespace xgc2_math::scene;
using xgc2_math::Pose3;
using Clock = std::chrono::steady_clock;
constexpr std::int64_t second = 1000000000;
constexpr std::uint64_t fnv_offset = 1469598103934665603ull;
std::uint64_t hashBytes(std::uint64_t hash, const std::uint8_t *bytes,
                        std::size_t n) {
  for (std::size_t i = 0; i < n; ++i)
    hash = (hash ^ bytes[i]) * 1099511628211ull;
  return hash;
}
double elapsed(Clock::time_point start) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start)
      .count();
}
double cpuMs() {
  return 1000.0 * static_cast<double>(std::clock()) / CLOCKS_PER_SEC;
}
double quantile(std::vector<double> samples, double q) {
  std::sort(samples.begin(), samples.end());
  return samples[std::min(
      samples.size() - 1,
      static_cast<std::size_t>(std::ceil(q * samples.size())) - 1)];
}

std::vector<Obstacle> clutter() {
  std::mt19937 generator(7);
  std::uniform_real_distribution<double> unit(0, 1);
  std::vector<Obstacle> obstacles;
  obstacles.reserve(600);
  for (int i = 0; i < 600; ++i) {
    ClutteredObstacle o;
    o.id = i;
    o.kind = static_cast<ClutteredObstacle::Kind>(i % 5 == 4 ? 5 : i % 5);
    o.position = {32 * unit(generator) - 16, 24 * unit(generator) - 12,
                  3 * unit(generator)};
    o.orientation =
        Eigen::AngleAxisd(6 * unit(generator), Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(0.4 * unit(generator), Eigen::Vector3d::UnitX());
    const double radius = 0.15 + 0.3 * unit(generator);
    o.scale =
        o.kind == ClutteredObstacle::kCube
            ? Eigen::Vector3d(0.5 + unit(generator), 0.5 + unit(generator),
                              0.5 + unit(generator))
            : Eigen::Vector3d(radius, radius, 1 + 2 * unit(generator));
    if (o.kind == ClutteredObstacle::kVPolytope) {
      o.scale = {0.5 + unit(generator), 0.5 + unit(generator),
                 1 + unit(generator)};
      o.vertices = {{-0.5, -0.5, -0.5}, {0.5, -0.5, -0.5}, {0.5, 0.5, -0.5},
                    {-0.5, 0.5, -0.5},  {-0.2, -0.1, 0.5}, {0.3, -0.2, 0.4},
                    {0.3, 0.2, 0.5},    {-0.2, 0.3, 0.4}};
    }
    obstacles.push_back(toObstacle(o));
  }
  return obstacles;
}

FleetManifest manifest(std::size_t sensors, bool raycast) {
  FleetManifest result;
  for (std::size_t i = 0; i < sensors; ++i) {
    SensorSpec spec;
    spec.name_space = "/sensor" + std::to_string(i);
    spec.body_id = static_cast<int>(i + 1);
    spec.pose_topic = "/truth" + spec.name_space;
    spec.config.mode =
        raycast ? SensorConfig::kRaycast : SensorConfig::kPenetrating;
    spec.config.range = 8;
    spec.config.h_fov_deg = 360;
    spec.config.v_fov_deg = 180;
    spec.config.h_res = 360;
    spec.config.v_res = 32;
    spec.config.surface_spacing = 0.1;
    spec.config.penetrating_keep_buried = !raycast;
    spec.publish_beams = raycast;
    result.robots.push_back(std::move(spec));
  }
  return result;
}

// Identical non-allocating sink work in both paths. Hash every delivered byte
// synchronously; latency includes the delivery and this consumer work. Separate
// cache lines avoid adding a false-sharing bottleneck to the measured pool.
struct DeliverySink : Sink {
  struct alignas(64) Sensor {
    std::uint64_t checksum = fnv_offset;
    std::size_t points = 0, beams = 0, scans = 0;
    double last_ms = 0;
  };
  explicit DeliverySink(std::size_t n) : sensors(n) {}
  std::vector<Sensor> sensors;
  Clock::time_point entry;
  void onPoints(std::size_t i, CloudView c) override {
    auto &s = sensors[i];
    s.checksum = hashBytes(s.checksum, c.data, c.byteSize());
    s.points += c.count;
    ++s.scans;
    s.last_ms = elapsed(entry);
  }
  void onBeams(std::size_t i, CloudView c) override {
    auto &s = sensors[i];
    s.checksum = hashBytes(s.checksum, c.data, c.byteSize());
    s.beams += c.count;
    s.last_ms = elapsed(entry);
  }
  void onReference(CloudView, ReferenceAction) override {
    throw std::logic_error("benchmark has no reference demand");
  }
  double lastDelivery() const {
    double last = 0;
    for (const auto &s : sensors)
      last = std::max(last, s.last_ms);
    return last;
  }
  void reset() {
    for (auto &s : sensors)
      s = Sensor{};
  }
};

// world_lidar_node.cpp: fresh/active arrays rebuilt per tick, per-vehicle heap
// objects, exact simulation schedules, and allocating beam messages. ROS
// transport is replaced by the same sink, leaving the fleet work comparable.
class OldFleet {
public:
  OldFleet(const FleetManifest &manifest, DeliverySink &sink,
           std::size_t scan_threads)
      : manifest_(manifest), sink_(sink), next_(manifest.robots.size()),
        poses_(next_.size()), stamps_(next_.size()), clouds_(next_.size()),
        pool_(scan_threads) {
    for (const auto &spec : manifest.robots)
      lidars_.push_back(std::make_unique<WorldLidar>(spec.config));
  }
  void setScene(const std::vector<Obstacle> &obstacles,
                std::uint64_t revision) {
    std::map<std::pair<double, bool>, std::shared_ptr<const LidarScene>>
        prepared;
    for (const auto &lidar : lidars_) {
      const auto &c = lidar->config();
      const auto key =
          std::make_pair(c.surface_spacing, c.penetrating_keep_buried);
      auto &scene = prepared[key];
      if (!scene) {
        const auto previous = scenes_.find(key);
        scene = previous == scenes_.end()
                    ? std::make_shared<const LidarScene>(obstacles, key.first,
                                                         key.second)
                    : std::make_shared<const LidarScene>(
                          obstacles, key.first, key.second, *previous->second);
      }
    }
    for (auto &lidar : lidars_) {
      const auto &c = lidar->config();
      lidar->setScene(
          prepared.at({c.surface_spacing, c.penetrating_keep_buried}));
    }
    scenes_ = std::move(prepared);
    revision_ = revision;
  }
  void setPose(std::size_t i, Pose3 pose, std::int64_t stamp) {
    poses_[i] = std::move(pose);
    stamps_[i] = stamp;
  }
  std::size_t threads() const { return pool_.threads(); }
  std::size_t tick(std::int64_t now) {
    std::vector<std::size_t> fresh, active;
    for (std::size_t i = 0; i < poses_.size(); ++i)
      if (freshObservationTime(stamps_[i] * 1e-9, now * 1e-9, 0.5))
        fresh.push_back(i);
    for (const auto i : fresh)
      if (next_[i] == 0 || now >= next_[i] - 1000)
        active.push_back(i);
    const std::vector<VehicleBody> bodies;
    for (const auto i : active) {
      if (next_[i] == 0)
        next_[i] = now;
      next_[i] += second / 10;
      if (next_[i] <= now)
        next_[i] = now + second / 10;
    }
    pool_.run(active.size(), [&](std::size_t job) {
      const auto i = active[job];
      const auto p = sensorWorldPose(poses_[i], Pose3{});
      const auto q = p.orientation.normalized();
      std::vector<VehicleBody> others;
      for (const auto &b : bodies)
        others.push_back(b);
      auto &cloud = clouds_[i];
      if (!manifest_.robots[i].publish_beams) {
        const auto count =
            lidars_[i]->scanInto(p.position, q, others, false, &cloud);
        sink_.onPoints(i, {cloud.data(), count, stamps_[i], 12, revision_});
      } else {
        const auto beams = lidars_[i]->scanWithBeams(p.position, q, others);
        const auto count = static_cast<std::size_t>(
            std::count(beams.hit.begin(), beams.hit.end(), std::uint8_t{1}));
        cloud.resize(12 * count);
        std::size_t hit = 0;
        for (std::size_t b = 0; b < beams.size(); ++b)
          if (beams.hit[b]) {
            const auto point = beams.point(b);
            const float xyz[3] = {static_cast<float>(point.x()),
                                  static_cast<float>(point.y()),
                                  static_cast<float>(point.z())};
            std::memcpy(cloud.data() + 12 * hit++, xyz, sizeof xyz);
          }
        sink_.onPoints(i, {cloud.data(), count, stamps_[i], 12, revision_});
        std::vector<std::uint8_t> bytes(32 * beams.size());
        for (std::size_t b = 0; b < beams.size(); ++b) {
          const auto &d = beams.direction[b];
          const float record[8] = {static_cast<float>(beams.origin.x()),
                                   static_cast<float>(beams.origin.y()),
                                   static_cast<float>(beams.origin.z()),
                                   static_cast<float>(d.x()),
                                   static_cast<float>(d.y()),
                                   static_cast<float>(d.z()),
                                   static_cast<float>(beams.range[b]),
                                   static_cast<float>(beams.hit[b])};
          std::memcpy(bytes.data() + 32 * b, record, sizeof record);
        }
        sink_.onBeams(i,
                      {bytes.data(), beams.size(), stamps_[i], 32, revision_});
      }
    });
    return active.size();
  }

private:
  const FleetManifest &manifest_;
  DeliverySink &sink_;
  std::vector<std::unique_ptr<WorldLidar>> lidars_;
  std::vector<std::int64_t> next_;
  std::vector<Pose3> poses_;
  std::vector<std::int64_t> stamps_;
  std::vector<std::vector<std::uint8_t>> clouds_;
  std::map<std::pair<double, bool>, std::shared_ptr<const LidarScene>> scenes_;
  std::uint64_t revision_ = 0;
  xgc2_world_lidar::ScanPool pool_;
};

Pose3 sensorPose(std::size_t i, std::size_t n, int tick) {
  const auto side =
      static_cast<std::size_t>(std::ceil(std::sqrt(static_cast<double>(n))));
  Pose3 pose;
  pose.position = {-16 + 30.0 * ((i % side) + 0.5) / side + 0.075 * tick,
                   -11 + 22.0 * ((i / side) + 0.5) / side, 1};
  return pose;
}
struct Result {
  double wall = 0, cpu = 0, allocations = 0, p50 = 0, p99 = 0, maximum = 0;
  std::size_t scans = 0, points = 0, beams = 0, threads = 0;
  std::uint64_t checksum = fnv_offset;
};
template <class Fleet>
Result measure(Fleet &fleet, DeliverySink &sink, std::size_t n) {
  for (std::size_t i = 0; i < n; ++i)
    fleet.setPose(i, sensorPose(i, n, 0), second);
  sink.entry = Clock::now();
  fleet.tick(second);
  sink.reset();
  std::vector<double> wall, latency;
  wall.reserve(100);
  latency.reserve(100);
  Result result;
  result.threads = fleet.threads();
  for (int tick = 0; tick < 100; ++tick) {
    const auto now = 2 * second + tick * second / 10;
    for (std::size_t i = 0; i < n; ++i)
      fleet.setPose(i, sensorPose(i, n, tick), now);
    allocation_counter::start();
    const auto cpu_start = cpuMs();
    sink.entry = Clock::now();
    const auto scans = fleet.tick(now);
    const double ms = elapsed(sink.entry);
    result.cpu += cpuMs() - cpu_start;
    const auto allocations = allocation_counter::stop();
    if (scans != n)
      throw std::runtime_error("benchmark missed a due scan");
    result.allocations += allocations;
    wall.push_back(ms);
    latency.push_back(sink.lastDelivery());
  }
  result.wall = std::accumulate(wall.begin(), wall.end(), 0.0) / 100;
  result.allocations /= 100;
  result.p50 = quantile(latency, 0.5);
  result.p99 = quantile(latency, 0.99);
  result.maximum = *std::max_element(latency.begin(), latency.end());
  for (const auto &s : sink.sensors) {
    result.scans += s.scans;
    result.points += s.points;
    result.beams += s.beams;
    result.checksum = hashBytes(
        result.checksum, reinterpret_cast<const std::uint8_t *>(&s.checksum),
        sizeof s.checksum);
  }
  return result;
}
void print(int run, const char *path, const char *mode, std::size_t n,
           const Result &r) {
  std::printf("fleet,%d,%s,%s,%zu,%zu,%.6f,%.6f,%.3f,%.6f,%.6f,%.6f,%zu,%zu,%"
              "zu,%016llx\n",
              run, path, mode, n, r.threads, r.wall, r.cpu, r.allocations,
              r.p50, r.p99, r.maximum, r.scans, r.points, r.beams,
              static_cast<unsigned long long>(r.checksum));
  std::fflush(stdout);
}
template <class Fleet>
void revisions(Fleet &fleet, const std::vector<Obstacle> &initial, int run,
               const char *path, const char *mode, std::size_t n) {
  std::vector<Obstacle> moved = initial;
  std::vector<double> wall, cpu;
  wall.reserve(40);
  cpu.reserve(40);
  for (std::uint64_t revision = 2; revision < 42; ++revision) {
    for (std::size_t i = 0; i < 5; ++i)
      moved[i].position.x() += 0.05;
    const auto start = Clock::now();
    const auto cpu_start = cpuMs();
    fleet.setScene(moved, revision);
    cpu.push_back(cpuMs() - cpu_start);
    wall.push_back(elapsed(start));
  }
  std::printf("revision,%d,%s,%s,%zu,40,%.6f,%.6f,%.6f,%.6f\n", run, path, mode,
              n, std::accumulate(wall.begin(), wall.end(), 0.0) / 40,
              std::accumulate(cpu.begin(), cpu.end(), 0.0) / 40,
              quantile(wall, 0.5), quantile(wall, 0.99));
  std::fflush(stdout);
}
void load(const char *stage, int run) {
  double averages[3]{};
  getloadavg(averages, 3);
  std::printf("load,%s,%d,%.2f,%.2f,%.2f\n", stage, run, averages[0],
              averages[1], averages[2]);
  std::fflush(stdout);
}

// The installed Math API does not expose reservation of its private near-body
// scratch. Record the first growth when a previously distant roster body
// enters range, separately from the steady fleet measurements.
void bodyCandidateGrowth() {
  auto m = manifest(1, true);
  m.robots[0].config.h_res = 1;
  m.robots[0].config.v_res = 1;
  m.robots[0].config.h_fov_deg = 90;
  m.body_roster = true;
  m.bodies = {{1, "/sensor0", "/truth/sensor0", 0.3},
              {7, "/neighbor", "/truth/neighbor", 0.3}};
  DeliverySink sink(1);
  Engine engine(m, sink);
  engine.setScene({}, 1);
  engine.setDemand(0, true);
  Pose3 self, other;
  other.position = {100, 0, 0};
  engine.setBodyPose(0, self, second);
  engine.setBodyPose(1, other, second);
  sink.entry = Clock::now();
  engine.tick(second);
  other.position.x() = 3;
  engine.setBodyPose(1, other, second + second / 10);
  allocation_counter::start();
  sink.entry = Clock::now();
  engine.tick(second + second / 10);
  const auto growth = allocation_counter::stop();
  allocation_counter::start();
  sink.entry = Clock::now();
  engine.tick(second + 2 * second / 10);
  const auto steady = allocation_counter::stop();
  std::printf("body_candidate_growth,first_near_tick_allocations,%zu,next_tick_"
              "allocations,%zu\n",
              growth, steady);
  std::fflush(stdout);
}
} // namespace

int main(int argc, char **argv) {
  try {
    const int repeats = argc >= 2 ? std::stoi(argv[1]) : 1;
    const int scan_threads = argc == 3 ? std::stoi(argv[2]) : 1;
    if (argc > 3 || repeats < 1 || scan_threads < 0)
      throw std::invalid_argument(
          "usage: bench_lidar_fleet [repetitions [scan_threads]]");
    const auto obstacles = clutter();
    bodyCandidateGrowth();
    std::printf("hardware,%u,obstacles,%zu,ticks,100,hz,10,moving,5\n",
                std::thread::hardware_concurrency(), obstacles.size());
    std::puts("fleet columns: "
              "kind,run,path,mode,sensors,threads,wall_mean_ms,cpu_total_ms,"
              "allocations_per_tick,last_sink_p50_ms,last_sink_p99_ms,last_"
              "sink_max_ms,scans,points,beams,checksum");
    std::puts("revision columns: "
              "kind,run,path,mode,sensors,revisions,wall_mean_ms,cpu_mean_ms,"
              "wall_p50_ms,wall_p99_ms");
    for (int run = 0; run < repeats; ++run) {
      load("start", run);
      for (const bool raycast : {false, true})
        for (const auto n : {std::size_t{20}, std::size_t{100}}) {
          const auto m = manifest(n, raycast);
          Result old, engine;
          const char *mode = raycast ? "raycast_beams" : "penetrating";
          EngineOptions options;
          options.worker_threads = static_cast<int>(
              scan_threads == 0
                  ? xgc2_world_lidar::defaultScanThreads(
                        n, std::thread::hardware_concurrency())
                  : std::min(n, static_cast<std::size_t>(scan_threads)));
          // Reverse each pair on alternate repetitions to expose order/load
          // bias.
          for (int order = 0; order < 2; ++order) {
            DeliverySink sink(n);
            if ((order + run) % 2 == 0) {
              OldFleet fleet(m, sink, options.worker_threads);
              fleet.setScene(obstacles, 1);
              old = measure(fleet, sink, n);
              print(run, "old", mode, n, old);
              revisions(fleet, obstacles, run, "old", mode, n);
            } else {
              Engine fleet(m, sink, options);
              fleet.setScene(obstacles, 1);
              for (std::size_t i = 0; i < n; ++i)
                fleet.setDemand(i, true);
              engine = measure(fleet, sink, n);
              print(run, "engine", mode, n, engine);
              revisions(fleet, obstacles, run, "engine", mode, n);
            }
          }
          if (old.checksum != engine.checksum || old.points != engine.points ||
              old.beams != engine.beams)
            throw std::runtime_error(
                "old and engine delivered different bytes/work");
          if (engine.allocations != 0)
            throw std::runtime_error("engine tick allocated after warm-up");
        }
      load("end", run);
    }
  } catch (const std::exception &error) {
    std::fprintf(stderr, "bench_lidar_fleet: %s\n", error.what());
    return 1;
  }
}
