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
      points_(lidars_.size()), beams_(lidars_.size()),
      beam_scans_(lidars_.size()), others_(lidars_.size()),
      next_due_(lidars_.size(), -1), schedule_epoch_(lidars_.size()),
      stamps_(lidars_.size()), schedule_index_(lidars_.size()),
      periods_(lidars_.size()), sensor_enabled_(lidars_.size(), 1),
      demand_(lidars_.size()), pose_valid_(lidars_.size()),
      with_bodies_(lidars_.size()), poses_(lidars_.size()),
      active_(lidars_.size()), body_poses_(manifest_.bodies.size()),
      body_stamps_(body_poses_.size()), body_valid_(body_poses_.size()),
      pool_(threadsFor(lidars_.size(), options.worker_threads)) {
  if (!std::isfinite(options.pose_timeout) || options.pose_timeout <= 0.0)
    throw std::invalid_argument(
        "world lidar needs frame_id=world and a positive finite pose_timeout");
  if (options.vehicle_bodies && !(std::isfinite(options.vehicle_body_radius) &&
                                  options.vehicle_body_radius > 0.0))
    throw std::invalid_argument("~vehicle_body_radius must be > 0");
  const auto body_count =
      manifest_.body_roster ? manifest_.bodies.size() : lidars_.size();
  body_snapshot_.reserve(body_count);
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

bool Engine::setScene(const std::vector<Obstacle> &obstacles,
                      std::uint64_t revision) {
  if (scenes_.ready() && revision == scenes_.revision())
    return false;
  if (!invalidateScene(revision))
    return false;
  scenes_.setScene(obstacles, revision);
  for (std::size_t i = 0; i < lidars_.size(); ++i) {
    const auto &scene = scenes_.scene(policies_[i]);
    lidars_[i]->setScene(scene);
    const auto bound = lidars_[i]->config().mode == SensorConfig::kPenetrating
                           ? scene->sampleCount() + body_point_bounds_[i]
                           : lidars_[i]->beamCount();
    points_[i].reserve(12 * bound);
    if (manifest_.robots[i].publish_beams)
      beams_[i].reserve(32 * lidars_[i]->beamCount());
  }
  return true;
}

bool Engine::invalidateScene(std::uint64_t revision) {
  if (!scenes_.invalidate(revision))
    return false;
  for (auto &data : points_)
    data.clear();
  for (auto &data : beams_)
    data.clear();
  return true;
}

const std::shared_ptr<const LidarScene> &
Engine::sensorScene(std::size_t sensor) const {
  return lidars_.at(sensor)->scene();
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
  return freshObservationTime(static_cast<double>(stamp) * 1e-9,
                              static_cast<double>(now) * 1e-9,
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

std::size_t Engine::tick(std::int64_t now) {
  if (last_tick_ >= 0 && now < last_tick_)
    std::fill(next_due_.begin(), next_due_.end(), -1);
  last_tick_ = now;
  if (!enabled_ || !scenes_.ready())
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
  pool_.run(count, this, [](void *context, std::size_t job) {
    auto &engine = *static_cast<Engine *>(context);
    engine.scan(engine.active_[job]);
  });
  return count;
}

void Engine::scan(std::size_t sensor) {
  const auto p = sensorWorldPose(poses_[sensor], xgc2_math::Pose3{});
  Eigen::Quaterniond q = p.orientation.normalized();
  if (options_.yaw_only) {
    const Eigen::Vector3d x = q * Eigen::Vector3d::UnitX();
    q = Eigen::AngleAxisd(std::atan2(x.y(), x.x()), Eigen::Vector3d::UnitZ());
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
    auto &scan = beam_scans_[sensor];
    lidars_[sensor]->scanBeamsInto(p.position, q, others, &scan);
    count = static_cast<std::size_t>(
        std::count(scan.hit.begin(), scan.hit.end(), std::uint8_t{1}));
    points.resize(12 * count);
    auto &beams = beams_[sensor];
    beams.resize(32 * scan.size());
    std::size_t hit = 0;
    for (std::size_t i = 0; i < scan.size(); ++i) {
      if (scan.hit[i])
        packPoint(scan.point(i), points.data() + 12 * hit++);
      const auto &d = scan.direction[i];
      const float record[8] = {static_cast<float>(scan.origin.x()),
                               static_cast<float>(scan.origin.y()),
                               static_cast<float>(scan.origin.z()),
                               static_cast<float>(d.x()),
                               static_cast<float>(d.y()),
                               static_cast<float>(d.z()),
                               static_cast<float>(scan.range[i]),
                               static_cast<float>(scan.hit[i])};
      std::memcpy(beams.data() + 32 * i, record, sizeof record);
    }
  }
  sink_.onPoints(
      sensor, {points.data(), count, stamps_[sensor], 12, scenes_.revision()});
  if (manifest_.robots[sensor].publish_beams &&
      lidars_[sensor]->config().mode != SensorConfig::kPenetrating)
    sink_.onBeams(sensor, {beams_[sensor].data(), beam_scans_[sensor].size(),
                           stamps_[sensor], 32, scenes_.revision()});
}

} // namespace xgc_scene::lidar
