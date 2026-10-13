#include "xgc_scene/lidar/engine.hpp"

#include <xgc2_math/scene/observation_contract.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <thread>
#include <utility>

namespace xgc_scene::lidar {
namespace {
using namespace xgc2_math::scene;

std::vector<SamplingPolicy> policiesOf(const FleetManifest &manifest) {
  std::vector<SamplingPolicy> policies;
  for (const auto &sensor : manifest.robots)
    policies.push_back(
        {sensor.config.surface_spacing, sensor.config.penetrating_keep_buried});
  if (manifest.reference)
    policies.push_back({manifest.reference->surface_spacing, false});
  return policies;
}

std::size_t threadsFor(std::size_t sensors, int requested) {
  return requested > 0
             ? std::min(static_cast<std::size_t>(requested),
                        std::max<std::size_t>(sensors, 1))
             : defaultScanThreads(sensors, std::thread::hardware_concurrency());
}

void packPoint(const Eigen::Vector3d &p, std::uint8_t *out) {
  const float xyz[3] = {static_cast<float>(p.x()), static_cast<float>(p.y()),
                        static_cast<float>(p.z())};
  std::memcpy(out, xyz, sizeof xyz);
}
} // namespace

Engine::Engine(FleetManifest manifest, Sink &sink, EngineOptions options)
    : manifest_(std::move(manifest)), sink_(sink), options_(options),
      scenes_(policiesOf(manifest_)), lidars_(manifest_.robots.size()),
      policies_(lidars_.size()), body_point_bounds_(lidars_.size()),
      prepared_generation_(lidars_.size()), points_(lidars_.size()),
      beams_(lidars_.size()), beam_scans_(lidars_.size()),
      others_(lidars_.size()), next_due_(lidars_.size(), -1),
      schedule_epoch_(lidars_.size()), stamps_(lidars_.size()),
      schedule_index_(lidars_.size()), periods_(lidars_.size()),
      sensor_enabled_(lidars_.size(), 1), demand_(lidars_.size()),
      pose_valid_(lidars_.size()), with_bodies_(lidars_.size()),
      poses_(lidars_.size()), active_(lidars_.size()),
      body_poses_(manifest_.bodies.size()), body_stamps_(body_poses_.size()),
      body_valid_(body_poses_.size()),
      pool_(threadsFor(std::count_if(manifest_.robots.begin(), manifest_.robots.end(),
                                    [](const auto& sensor) { return sensor.backend != "gpu"; }),
                       options.worker_threads)) {
  if (!std::isfinite(options.pose_timeout) || options.pose_timeout <= 0.0)
    throw std::invalid_argument(
        "world lidar needs frame_id=world and a positive finite pose_timeout");
  if (options.vehicle_bodies && !(std::isfinite(options.vehicle_body_radius) &&
                                  options.vehicle_body_radius > 0.0))
    throw std::invalid_argument("~vehicle_body_radius must be > 0");
  const auto body_count =
      manifest_.body_roster ? manifest_.bodies.size() : lidars_.size();
  body_snapshot_.reserve(body_count);
  gpu_active_.resize(lidars_.size());
  cpu_fallback_.resize(lidars_.size(), false);
  observation_cache_.resize(lidars_.size());
  point_scratch_.resize(lidars_.size());
  std::vector<std::shared_ptr<const BeamPattern>> patterns;
  for (std::size_t i = 0; i < lidars_.size(); ++i) {
    auto config = manifest_.robots[i].config;
    resolveSensorConfig(&config);
    const auto pattern =
        std::find_if(patterns.begin(), patterns.end(),
                     [&](const auto &p) { return p->matches(config); });
    const auto shared =
        pattern == patterns.end() ? BeamPattern::make(config) : *pattern;
    if (pattern == patterns.end())
      patterns.push_back(shared);
    lidars_[i].emplace(config, shared);
    policies_[i] = scenes_.policyIndex(
        {config.surface_spacing, config.penetrating_keep_buried});
    const double rate = manifest_.robots[i].rate_hz;
    if (!std::isfinite(rate) || rate <= 0.0 || rate > 100.0)
      throw std::invalid_argument("invalid fleet sensor rate");
    periods_[i] = 1000000000.0L / rate;
    with_bodies_[i] =
        options.vehicle_bodies ||
        (manifest_.body_roster && config.mode != SensorConfig::kPenetrating);
    others_[i].reserve(body_count);
    if (with_bodies_[i] && config.mode == SensorConfig::kPenetrating) {
      if (manifest_.body_roster) {
        for (const auto &body : manifest_.bodies)
          if (body.id != manifest_.robots[i].body_id)
            body_point_bounds_[i] +=
                sampleSurface(
                    Obstacle::sphere(Eigen::Vector3d::Zero(), body.radius),
                    config.surface_spacing)
                    .size();
      } else if (body_count > 0) {
        body_point_bounds_[i] =
            (body_count - 1) *
            sampleSurface(Obstacle::sphere(Eigen::Vector3d::Zero(),
                                           options.vehicle_body_radius),
                          config.surface_spacing)
                .size();
      }
    }
  }
}

bool Engine::setPointCloud(const std::vector<std::uint8_t>& xyz, std::uint64_t revision) {
  point_map_ = std::make_shared<PointMap>(xyz);
  return setScene({}, revision);
}

bool Engine::setScene(const std::vector<Obstacle> &obstacles,
                      std::uint64_t revision) {
  if (scenes_.ready() && revision == scenes_.revision())
    return false;
  if (!invalidateScene(revision))
    return false;
  gpu_maps_.clear();
  scenes_.setScene(point_map_ ? std::vector<Obstacle>{} : obstacles, revision);
  ++preparation_generation_;
  pending_sensors_ = lidars_.size();
  if (manifest_.reference) {
    const auto &reference = scenes_.scene(
        scenes_.policyIndex({manifest_.reference->surface_spacing, false}));
    reference_bytes_.resize(12 * reference->sampleCount());
    for (std::size_t i = 0; i < reference->sampleCount(); ++i)
      packPoint(reference->samples()[i], reference_bytes_.data() + 12 * i);
    reference_dirty_ = true;
  }
  return true;
}

bool Engine::invalidateScene(std::uint64_t revision) {
  if (!scenes_.invalidate(revision))
    return false;
  reference_dirty_ = false;
  reference_clear_pending_ = bool(manifest_.reference);
  reference_bytes_.clear();
  // Sink views expire with their callbacks and unready truth suppresses scans.
  // The next scan overwrites reused outputs; walking their headers is needless.
  return true;
}

const std::shared_ptr<const LidarScene> &
Engine::sensorScene(std::size_t sensor) const {
  return scenes_.scene(policies_.at(sensor));
}
const std::shared_ptr<const BeamPattern> &
Engine::beamPattern(std::size_t sensor) const {
  return lidars_.at(sensor)->beamPattern();
}

void Engine::setEnabled(std::size_t sensor, bool enabled) {
  sensor_enabled_.at(sensor) = enabled;
}
void Engine::setDemand(std::size_t sensor, bool demand) {
  demand_.at(sensor) = demand;
}

bool Engine::setPose(std::size_t sensor, const xgc2_math::Pose3 &pose,
                     std::int64_t stamp, const std::string &frame) {
  const bool valid = validObservationPose(pose, frame, "world");
  pose_valid_.at(sensor) = valid;
  poses_[sensor] = pose;
  stamps_[sensor] = stamp;
  return valid;
}

bool Engine::setBodyPose(std::size_t body, const xgc2_math::Pose3 &pose,
                         std::int64_t stamp, const std::string &frame) {
  const bool valid = validObservationPose(pose, frame, "world");
  body_valid_.at(body) = valid;
  body_poses_[body] = pose;
  body_stamps_[body] = stamp;
  for (std::size_t i = 0; i < lidars_.size(); ++i)
    if (manifest_.robots[i].pose_topic == manifest_.bodies[body].pose_topic)
      setPose(i, pose, stamp, frame);
  return valid;
}

bool Engine::fresh(std::int64_t stamp, std::int64_t now) const {
  if (stamp < 0 || now < stamp)
    return false;
  // Subtract in nanoseconds before converting to Math's seconds contract:
  // large absolute clock values must not round a future sample into the past.
  return freshObservationTime(0.0, static_cast<double>(now - stamp) * 1e-9,
                              options_.pose_timeout);
}

void Engine::advance(std::size_t sensor, std::int64_t now) {
  if (next_due_[sensor] < 0) {
    schedule_epoch_[sensor] = now;
    schedule_index_[sensor] = 1;
  } else {
    ++schedule_index_[sensor];
  }
  // Compute from the phase origin rather than repeatedly rounding a period.
  long double next = schedule_epoch_[sensor] +
                     std::ceil(schedule_index_[sensor] * periods_[sensor]);
  if (next <= now) {
    schedule_epoch_[sensor] = now;
    schedule_index_[sensor] = 1;
    next = now + std::ceil(periods_[sensor]);
  }
  next_due_[sensor] = next >= std::numeric_limits<std::int64_t>::max()
                          ? std::numeric_limits<std::int64_t>::max()
                          : static_cast<std::int64_t>(next);
}

void Engine::prepareSensors(std::size_t count) {
  for (std::size_t job = 0; job < count; ++job) {
    const auto sensor = active_[job];
    if (prepared_generation_[sensor] == preparation_generation_)
      continue;
    const auto &scene = scenes_.scene(policies_[sensor]);
    lidars_[sensor]->setScene(scene);
    const auto bound =
        lidars_[sensor]->config().mode == SensorConfig::kPenetrating
            ? scene->sampleCount() + body_point_bounds_[sensor]
            : lidars_[sensor]->beamCount();
    points_[sensor].reserve(12 * bound);
    if (manifest_.robots[sensor].publish_beams)
      beams_[sensor].reserve(32 * lidars_[sensor]->beamCount());
    prepared_generation_[sensor] = preparation_generation_;
    --pending_sensors_;
  }
}

std::size_t Engine::tick(std::int64_t now) {
  if (last_tick_ >= 0 && now < last_tick_)
    std::fill(next_due_.begin(), next_due_.end(), -1);
  last_tick_ = now;
  if (!enabled_)
    return 0;
  deliverReference(now);
  if (!scenes_.ready())
    return 0;
  bool body_truth_ready = true;
  body_snapshot_.clear();
  if (manifest_.body_roster) {
    for (std::size_t i = 0; i < body_poses_.size(); ++i) {
      if (!body_valid_[i] || !fresh(body_stamps_[i], now)) {
        body_truth_ready = false;
        break;
      }
      body_snapshot_.push_back({manifest_.bodies[i].id, body_poses_[i].position,
                                manifest_.bodies[i].radius});
    }
  } else if (options_.vehicle_bodies) {
    for (std::size_t i = 0; i < poses_.size(); ++i)
      if (pose_valid_[i] && fresh(stamps_[i], now))
        body_snapshot_.push_back({manifest_.robots[i].body_id,
                                  poses_[i].position,
                                  options_.vehicle_body_radius});
  }
  std::size_t count = 0;
  for (std::size_t i = 0; i < lidars_.size(); ++i) {
    if (!sensor_enabled_[i] || !demand_[i] || !pose_valid_[i] ||
        (next_due_[i] >= 0 && now < next_due_[i]) || !fresh(stamps_[i], now) ||
        (with_bodies_[i] && !body_truth_ready))
      continue;
    active_[count++] = i;
    advance(i, now);
  }
  if (pending_sensors_ != 0)
    prepareSensors(count);
  std::size_t cpu_count=0;gpu_count_=0;
  for(std::size_t job=0;job<count;++job){const auto i=active_[job];
    if(manifest_.robots[i].backend != "cpu" && !cpu_fallback_[i]) gpu_active_[gpu_count_++]=i;
    else active_[cpu_count++]=i;
  }
  pool_.run(cpu_count, this, [](void *context, std::size_t job) {
    auto &engine = *static_cast<Engine *>(context);
    engine.scan(engine.active_[job]);
  }, [](void* context){auto& engine=*static_cast<Engine*>(context);for(std::size_t j=0;j<engine.gpu_count_;++j)engine.scan(engine.gpu_active_[j]);});
  return count;
}

void Engine::deliverReference(std::int64_t now) {
  if (reference_clear_pending_) {
    sink_.onReference({nullptr, 0, now, 12, scenes_.revision()},
                      ReferenceAction::Clear);
    reference_clear_pending_ = false;
  }
  if (scenes_.ready() && reference_dirty_ && reference_demand_) {
    sink_.onReference({reference_bytes_.data(), reference_bytes_.size() / 12,
                       now, 12, scenes_.revision()},
                      ReferenceAction::Publish);
    reference_dirty_ = false;
  }
}

void Engine::scan(std::size_t sensor) {
  const auto p = sensorWorldPose(poses_[sensor], xgc2_math::Pose3{});
  Eigen::Quaterniond q = p.orientation.normalized();
  if (options_.yaw_only) {
    const Eigen::Vector3d x = q * Eigen::Vector3d::UnitX();
    q = Eigen::AngleAxisd(std::atan2(x.y(), x.x()), Eigen::Vector3d::UnitZ());
  }
  const bool reusable = !with_bodies_[sensor] && !manifest_.robots[sensor].publish_beams &&
      lidars_[sensor]->config().noise_std == 0;
  auto& cache=observation_cache_[sensor];
  if(reusable && cache.generation==preparation_generation_ &&
     (cache.pose.position.array()==p.position.array()).all() &&
     (cache.pose.orientation.coeffs().array()==q.coeffs().array()).all()) {
    sink_.onPoints(sensor,{points_[sensor].data(),points_[sensor].size()/12,stamps_[sensor],12,scenes_.revision()});
    return;
  }
  const auto remember=[&]{if(reusable){cache.pose={p.position,q};cache.generation=preparation_generation_;}};
  const auto& spec=manifest_.robots[sensor];
  if(spec.backend != "cpu" && !cpu_fallback_[sensor]) {
    try {
      if (spec.config.mode == SensorConfig::kDepthFrustum || spec.config.noise_std != 0 ||
          spec.publish_beams || with_bodies_[sensor])
        throw std::runtime_error("The GPU point renderer does not implement depth, noise, beams or dynamic bodies");
      auto& gpu=gpu_maps_[policies_[sensor]];
      if(!gpu) {
        if (point_map_) gpu.reset(new GpuMap(point_map_->xyz(),spec.config));
        else gpu.reset(new GpuMap(sensorScene(sensor),spec.config));
      }
      gpu->scan(spec.config,{p.position,q},double(stamps_[sensor])*1e-9,points_[sensor]);
      remember();
      sink_.onPoints(sensor,{points_[sensor].data(),points_[sensor].size()/12,stamps_[sensor],12,scenes_.revision()});
      return;
    } catch(const std::exception& error) { if(spec.backend != "auto") throw; cpu_fallback_[sensor]=true; }
  }
  if (point_map_) {
    point_map_->scan(spec.config, p.position, q, point_scratch_[sensor], points_[sensor]);
    remember();
    sink_.onPoints(sensor, {points_[sensor].data(), points_[sensor].size()/12,
                           stamps_[sensor], 12, scenes_.revision()});
    return;
  }
  auto &others = others_[sensor];
  others.clear();
  if (with_bodies_[sensor])
    for (const auto &body : body_snapshot_)
      if (body.id != manifest_.robots[sensor].body_id)
        others.push_back(body);
  auto &points = points_[sensor];
  std::size_t count = 0;
  if (!manifest_.robots[sensor].publish_beams ||
      lidars_[sensor]->config().mode == SensorConfig::kPenetrating) {
    count = lidars_[sensor]->scanInto(p.position, q, others, false, &points);
  } else {
    auto &scan = beam_scans_[sensor].scan;
    lidars_[sensor]->scanBeamsInto(p.position, q, others, &scan);
    // Byte writes can alias scan storage from the compiler's point of view.
    // Local headers avoid reloading origins and pointers for every record.
    const auto beam_count = scan.size();
    const Eigen::Vector3d origin = scan.origin;
    const auto *directions = scan.direction.data();
    const auto *ranges = scan.range.data();
    const auto *hits = scan.hit.data();
    count = static_cast<std::size_t>(
        std::count(scan.hit.begin(), scan.hit.end(), std::uint8_t{1}));
    points.resize(12 * count);
    auto *point_bytes = points.data();
    std::size_t hit = 0;
    for (std::size_t i = 0; i < beam_count; ++i)
      if (hits[i])
        packPoint(origin + ranges[i] * directions[i], point_bytes + 12 * hit++);
    sink_.onPoints(sensor, {points.data(), count, stamps_[sensor], 12,
                            scenes_.revision()});
    auto &beams = beams_[sensor];
    beams.resize(32 * beam_count);
    auto *beam_bytes = beams.data();
    const float ox = static_cast<float>(origin.x());
    const float oy = static_cast<float>(origin.y());
    const float oz = static_cast<float>(origin.z());
    for (std::size_t i = 0; i < beam_count; ++i) {
      const auto &d = directions[i];
      const float record[8] = {ox,
                               oy,
                               oz,
                               static_cast<float>(d.x()),
                               static_cast<float>(d.y()),
                               static_cast<float>(d.z()),
                               static_cast<float>(ranges[i]),
                               static_cast<float>(hits[i])};
      std::memcpy(beam_bytes + 32 * i, record, sizeof record);
    }
    sink_.onBeams(sensor, {beams.data(), beam_count, stamps_[sensor], 32,
                           scenes_.revision()});
    return;
  }
  remember();
  sink_.onPoints(
      sensor, {points.data(), count, stamps_[sensor], 12, scenes_.revision()});
}

} // namespace xgc_scene::lidar
