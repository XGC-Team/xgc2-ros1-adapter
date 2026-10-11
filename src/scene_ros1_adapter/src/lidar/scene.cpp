#include "xgc_scene/lidar/scene.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace xgc_scene::lidar {

SceneCache::SceneCache(std::vector<SamplingPolicy> policies) {
  for (const auto policy : policies)
    if (std::find(policies_.begin(), policies_.end(), policy) ==
        policies_.end())
      policies_.push_back(policy);
  scenes_.resize(policies_.size());
}

bool SceneCache::invalidate(std::uint64_t revision) {
  if (have_revision_ && revision < revision_)
    return false;
  revision_ = revision;
  have_revision_ = true;
  ready_ = false;
  return true;
}

bool SceneCache::setScene(
    const std::vector<xgc2_math::scene::Obstacle> &obstacles,
    std::uint64_t revision) {
  if (have_revision_ &&
      (revision < revision_ || (revision == revision_ && ready_)))
    return false;
  invalidate(revision);
  std::vector<std::shared_ptr<const xgc2_math::scene::LidarScene>> prepared;
  prepared.reserve(policies_.size());
  for (std::size_t i = 0; i < policies_.size(); ++i) {
    const auto policy = policies_[i];
    using xgc2_math::scene::LidarScene;
    prepared.push_back(
        scenes_[i]
            ? std::make_shared<const LidarScene>(
                  obstacles, policy.spacing, policy.keep_buried, *scenes_[i])
            : std::make_shared<const LidarScene>(obstacles, policy.spacing,
                                                 policy.keep_buried));
  }
  scenes_ = std::move(prepared);
  ready_ = true;
  return true;
}

std::size_t SceneCache::policyIndex(SamplingPolicy policy) const {
  const auto found = std::find(policies_.begin(), policies_.end(), policy);
  if (found == policies_.end())
    throw std::invalid_argument("unknown lidar sampling policy");
  return static_cast<std::size_t>(found - policies_.begin());
}

const std::shared_ptr<const xgc2_math::scene::LidarScene> &
SceneCache::scene(std::size_t policy) const {
  return scenes_.at(policy);
}

} // namespace xgc_scene::lidar
