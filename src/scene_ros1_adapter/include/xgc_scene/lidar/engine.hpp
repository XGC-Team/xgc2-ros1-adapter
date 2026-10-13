#pragma once

#include "xgc_scene/lidar/manifest.hpp"
#include "xgc_scene/lidar/gpu.hpp"
#include <map>
#include "xgc_scene/lidar/scan_pool.hpp"
#include "xgc_scene/lidar/scene.hpp"

#include <xgc2_math/scene/world_lidar.hpp>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace xgc_scene::lidar {

// Borrowed only during the sink callback. Host-order float32 records in world
// coordinates: points x/y/z (12 bytes), beams x/y/z/dx/dy/dz/range/hit (32).
struct CloudView {
  const std::uint8_t *data = nullptr;
  std::size_t count = 0;
  std::int64_t stamp_ns = 0;
  std::uint32_t point_step = 12;
  std::uint64_t revision = 0;
  std::size_t byteSize() const { return count * point_step; }
};

enum class ReferenceAction { Publish, Clear };

class Sink {
public:
  virtual ~Sink() = default;
  // Called concurrently for different sensors on pool workers or the tick
  // thread. For one sensor points precede beams. All deliveries finish before
  // tick returns. A sink must not call engine methods from a callback.
  virtual void onPoints(std::size_t sensor, CloudView cloud) = 0;
  virtual void onBeams(std::size_t sensor, CloudView cloud) = 0;
  // Called only on the tick thread. Clear invalidates the edge's latched cloud,
  // including on re-enable while awaiting scene truth; Publish delivers the
  // exterior map once per installed revision, when reference demand exists.
  virtual void onReference(CloudView cloud, ReferenceAction action) = 0;
};

struct EngineOptions {
  int worker_threads =
      0; // <= 0: defaultScanThreads, positive: capped by fleet size
  double pose_timeout = 0.5;
  bool yaw_only = false;
  bool vehicle_bodies = false; // explicit penetrating/legacy opt-in
  double vehicle_body_radius = 0.3;
};

// One owner thread calls all methods, including tick; calls must not overlap.
// The owner supplies simulation nanoseconds, demand and the latest received
// world-frame samples. No wall timers or ROS types exist. Unchanged static observations reuse bytes.
// The manifest and body membership are frozen for this object's lifetime.
class Engine {
public:
  Engine(FleetManifest manifest, Sink &sink, EngineOptions options = {});
  Engine(const Engine &) = delete;
  Engine &operator=(const Engine &) = delete;

  const FleetManifest &manifest() const { return manifest_; }
  std::size_t threads() const { return pool_.threads(); }
  bool sceneReady() const { return scenes_.ready(); }
  // Compiles shared geometry immediately. Sensor scratch and output buffers
  // are prepared only when that sensor next has a due, demanded scan.
  bool setScene(const std::vector<xgc2_math::scene::Obstacle> &obstacles,
                std::uint64_t revision);
  // Waiting for truth or a refused/removed source invalidates observations.
  // Retains the last compiled geometry only for a later incremental build.
  bool invalidateScene(std::uint64_t revision);
  const std::shared_ptr<const xgc2_math::scene::LidarScene> &
  sensorScene(std::size_t sensor) const;
  const std::shared_ptr<const xgc2_math::scene::BeamPattern> &
  beamPattern(std::size_t sensor) const;

  void setEnabled(bool enabled) { enabled_ = enabled; }
  void setEnabled(std::size_t sensor, bool enabled);
  void setDemand(std::size_t sensor, bool demand);
  void setReferenceDemand(bool demand) { reference_demand_ = demand; }
  bool setPose(std::size_t sensor, const xgc2_math::Pose3 &pose,
               std::int64_t stamp_ns, const std::string &frame = "world");
  // body is an index into manifest().bodies. Matching truth pose topics also
  // supply sensor poses; explicit sensor topics retain their separate samples.
  bool setBodyPose(std::size_t body, const xgc2_math::Pose3 &pose,
                   std::int64_t stamp_ns, const std::string &frame = "world");
  // At most one scan per due, enabled, demanded sensor. Missed periods are
  // skipped; a backwards clock resets schedules. Output stamps are pose stamps.
  std::size_t tick(std::int64_t sim_now_ns);

private:
  void prepareSensors(std::size_t count);
  void scan(std::size_t sensor);
  void advance(std::size_t sensor, std::int64_t now);
  bool fresh(std::int64_t stamp, std::int64_t now) const;
  void deliverReference(std::int64_t now);

  const FleetManifest manifest_;
  Sink &sink_;
  const EngineOptions options_;
  SceneCache scenes_;
  // Fixed-size contiguous objects, including each non-movable WorldLidar.
  // optional permits in-place construction with heterogeneous configurations.
  std::vector<std::optional<xgc2_math::scene::WorldLidar>> lidars_;
  std::vector<std::size_t> policies_, body_point_bounds_;
  std::vector<std::uint64_t> prepared_generation_;
  std::uint64_t preparation_generation_ = 0;
  std::size_t pending_sensors_ = 0;
  std::vector<std::vector<std::uint8_t>> points_, beams_;
  // Math updates four vector end pointers for every beam. Adjacent sensors
  // must not write those headers on the same cache line from different workers.
  struct alignas(64) SensorBeams {
    xgc2_math::scene::BeamScan scan;
  };
  std::vector<SensorBeams> beam_scans_;
  std::vector<std::vector<xgc2_math::scene::VehicleBody>> others_;

  // Hot fields are separate contiguous arrays; tick never grows these arrays.
  std::vector<std::int64_t> next_due_, schedule_epoch_, stamps_;
  std::vector<std::uint64_t> schedule_index_;
  std::vector<long double> periods_;
  std::vector<std::uint8_t> sensor_enabled_, demand_, pose_valid_, with_bodies_;
  std::vector<xgc2_math::Pose3> poses_;
  std::vector<std::size_t> active_, gpu_active_;
  std::size_t gpu_count_ = 0;
  std::vector<std::uint8_t> cpu_fallback_;
  struct alignas(64) ObservationCache {
    xgc2_math::Pose3 pose;
    std::uint64_t generation = 0;
  };
  std::vector<ObservationCache> observation_cache_;
  std::map<std::size_t,std::unique_ptr<GpuMap>> gpu_maps_;
  std::vector<xgc2_math::Pose3> body_poses_;
  std::vector<std::int64_t> body_stamps_;
  std::vector<std::uint8_t> body_valid_;
  std::vector<xgc2_math::scene::VehicleBody> body_snapshot_;
  std::vector<std::uint8_t> reference_bytes_;
  bool reference_demand_ = false;
  bool reference_dirty_ = false;
  bool reference_clear_pending_ = false;
  std::int64_t last_tick_ = -1;
  bool enabled_ = true;
  // Declared last so destruction joins workers before sensor storage dies.
  ScanPool pool_;
};

} // namespace xgc_scene::lidar
