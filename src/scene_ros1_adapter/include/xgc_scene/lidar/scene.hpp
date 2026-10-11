#pragma once

#include <xgc2_math/scene/lidar_scene.hpp>
#include <xgc2_math/scene/scene_conversion.hpp>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace xgc_scene::lidar {

struct SamplingPolicy {
  double spacing = 0.1;
  bool keep_buried = false;
  bool operator==(const SamplingPolicy &other) const {
    return spacing == other.spacing && keep_buried == other.keep_buried;
  }
};

// A flat, frozen set of sampling policies. Revision work is cold: prepare all
// policies before installing any. A refusal leaves the fleet unready, while the
// previous immutable scenes remain available to the next incremental build.
class SceneCache {
public:
  explicit SceneCache(std::vector<SamplingPolicy> policies);
  bool setScene(const std::vector<xgc2_math::scene::Obstacle> &obstacles,
                std::uint64_t revision);
  bool invalidate(std::uint64_t revision);
  bool ready() const { return ready_; }
  std::uint64_t revision() const { return revision_; }
  std::size_t policyIndex(SamplingPolicy policy) const;
  const std::shared_ptr<const xgc2_math::scene::LidarScene> &
  scene(std::size_t policy) const;

private:
  std::vector<SamplingPolicy> policies_;
  std::vector<std::shared_ptr<const xgc2_math::scene::LidarScene>> scenes_;
  std::uint64_t revision_ = 0;
  bool have_revision_ = false;
  bool ready_ = false;
};

} // namespace xgc_scene::lidar
