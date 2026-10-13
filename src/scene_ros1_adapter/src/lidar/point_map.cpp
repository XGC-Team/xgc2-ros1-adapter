#include "xgc_scene/lidar/point_map.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace xgc_scene::lidar {
PointMap::PointMap(const std::vector<std::uint8_t>& bytes) {
  if (bytes.size() % 12) throw std::invalid_argument("incomplete scene XYZ record");
  points_.resize(bytes.size() / 12);
  if (!bytes.empty()) std::memcpy(points_.data(), bytes.data(), bytes.size());
  for (const auto& point : points_)
    for (float value : point)
      if (!std::isfinite(value)) throw std::invalid_argument("non-finite scene point");
  nodes_.reserve(points_.size() / 32 + 1);
  if (!points_.empty()) build(0, points_.size());
  xyz_.resize(points_.size() * 3);
  if (!xyz_.empty()) std::memcpy(xyz_.data(), points_.data(), bytes.size());
  points_.clear();
  points_.shrink_to_fit();
}

std::size_t PointMap::build(std::size_t first, std::size_t last) {
  Node node{points_[first], points_[first], first, last};
  for (auto i = first + 1; i < last; ++i)
    for (int axis = 0; axis < 3; ++axis) {
      node.lo[axis] = std::min(node.lo[axis], points_[i][axis]);
      node.hi[axis] = std::max(node.hi[axis], points_[i][axis]);
    }
  const auto index = nodes_.size();
  nodes_.push_back(node);
  if (last - first > 64) {
    int axis = 0;
    for (int i = 1; i < 3; ++i)
      if (node.hi[i] - node.lo[i] > node.hi[axis] - node.lo[axis]) axis = i;
    const auto middle = first + (last - first) / 2;
    std::nth_element(points_.begin() + first, points_.begin() + middle,
                     points_.begin() + last,
                     [axis](const auto& a, const auto& b) { return a[axis] < b[axis]; });
    const auto left = build(first, middle), right = build(middle, last);
    nodes_[index].left = left;
    nodes_[index].right = right;
  }
  return index;
}

void PointMap::query(std::size_t index, const Eigen::Vector3d& center, double radius2,
                     std::vector<std::size_t>& output) const {
  const auto& node = nodes_[index];
  double distance2 = 0;
  for (int axis = 0; axis < 3; ++axis) {
    const double delta = std::max({double(node.lo[axis]) - center[axis],
                                   center[axis] - double(node.hi[axis]), 0.0});
    distance2 += delta * delta;
  }
  if (distance2 > radius2) return;
  if (node.left) {
    query(node.left, center, radius2, output);
    query(node.right, center, radius2, output);
  } else {
    for (auto i = node.first; i < node.last; ++i) {
      const auto* p = xyz_.data() + 3*i;
      if ((Eigen::Vector3d(p[0], p[1], p[2]) - center).squaredNorm() <= radius2)
        output.push_back(i);
    }
  }
}

void PointMap::scan(const xgc2_math::scene::SensorConfig& config,
                    const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
                    Scratch& scratch, std::vector<std::uint8_t>& output) const {
  using xgc2_math::scene::SensorConfig;
  constexpr double pi = 3.14159265358979323846;
  scratch.candidates.clear();
  output.clear();
  if (xyz_.empty()) return;
  query(0, position, config.range * config.range, scratch.candidates);
  const bool penetrating = config.mode == SensorConfig::kPenetrating;
  const auto rotation = orientation.normalized().toRotationMatrix();
  const double half_h = config.h_fov_deg * pi / 360, half_v = config.v_fov_deg * pi / 360;
  if (!penetrating) {
    const auto cells = std::size_t(config.h_res) * config.v_res;
    scratch.hits.clear();
    scratch.depth.assign(cells, std::numeric_limits<double>::infinity());
  }
  output.reserve(scratch.candidates.size() * 12);
  const auto append = [&](std::size_t index) {
    const auto offset = output.size();
    output.resize(offset + 12);
    std::memcpy(output.data() + offset, xyz_.data()+3*index, 12);
  };
  for (auto index : scratch.candidates) {
    const auto* p = xyz_.data()+3*index;
    const Eigen::Vector3d delta = Eigen::Vector3d(p[0], p[1], p[2]) - position;
    const double distance = delta.norm();
    if (distance == 0 || distance < config.min_range) continue;
    const Eigen::Vector3d local = rotation.transpose() * delta;
    const double azimuth = std::atan2(local.y(), local.x());
    const double elevation = std::atan2(local.z(), std::hypot(local.x(), local.y()));
    if (std::abs(azimuth) > half_h || std::abs(elevation) > half_v) continue;
    if (config.penetrating_heading_crop &&
        (delta.dot(rotation.col(0)) / distance < config.heading_cos_min ||
         std::abs(delta.z()) > config.range * config.vertical_slab_tan)) continue;
    if (penetrating) append(index);
    else {
      const auto h = std::min(config.h_res - 1, int((azimuth + half_h) / (2 * half_h) * config.h_res));
      const auto v = std::min(config.v_res - 1, int((elevation + half_v) / (2 * half_v) * config.v_res));
      const auto cell = std::size_t(v) * config.h_res + h;
      scratch.hits.push_back({index,cell,distance});
      const double cover = std::atan2(config.surface_spacing * 0.5, distance);
      const int cover_h = int(std::ceil(cover / (2 * half_h / config.h_res)));
      const int cover_v = int(std::ceil(cover / (2 * half_v / config.v_res)));
      for (int y = std::max(0,v-cover_v); y <= std::min(config.v_res-1,v+cover_v); ++y)
        for (int x = std::max(0,h-cover_h); x <= std::min(config.h_res-1,h+cover_h); ++x) {
          auto& depth = scratch.depth[std::size_t(y)*config.h_res+x];
          depth = std::min(depth,distance);
        }
    }
  }
  if (!penetrating)
    for (const auto& hit : scratch.hits)
      if (hit.distance <= scratch.depth[hit.cell] + config.surface_spacing) append(hit.index);
}
}  // namespace xgc_scene::lidar
