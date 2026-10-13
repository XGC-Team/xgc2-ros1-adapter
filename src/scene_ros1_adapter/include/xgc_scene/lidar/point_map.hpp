#pragma once

#include <xgc2_math/scene/sensor_model.hpp>
#include <Eigen/Geometry>
#include <array>
#include <cstdint>
#include <vector>

namespace xgc_scene::lidar {

// Immutable spatial index for a saved scene cloud. One map is shared by the
// fleet; each sensor owns its output and scratch, with no per-scan map copies.
class PointMap {
 public:
  struct Scratch {
    struct Hit { std::size_t index, cell; double distance; };
    std::vector<std::size_t> candidates;
    std::vector<Hit> hits;
    std::vector<double> depth;
  };
  explicit PointMap(const std::vector<std::uint8_t>& xyz);
  const std::vector<float>& xyz() const { return xyz_; }
  void scan(const xgc2_math::scene::SensorConfig&, const Eigen::Vector3d&,
            const Eigen::Quaterniond&, Scratch&, std::vector<std::uint8_t>&) const;

 private:
  struct Node {
    std::array<float, 3> lo, hi;
    std::size_t first, last, left = 0, right = 0;
  };
  std::vector<std::array<float, 3>> points_;
  std::vector<float> xyz_;
  std::vector<Node> nodes_;
  std::size_t build(std::size_t first, std::size_t last);
  void query(std::size_t, const Eigen::Vector3d&, double,
             std::vector<std::size_t>&) const;
};
}  // namespace xgc_scene::lidar
