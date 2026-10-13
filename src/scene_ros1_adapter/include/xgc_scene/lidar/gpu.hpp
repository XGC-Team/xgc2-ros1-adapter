#pragma once
#include <xgc2_math/scene/world_lidar.hpp>
#include <xgc2_math/scene/observation_contract.hpp>
#include <memory>
#include <vector>
#include <cstdint>
namespace xgc_scene::lidar {
class GpuMap {
public:
  explicit GpuMap(const std::shared_ptr<const xgc2_math::scene::LidarScene>& scene,
                  const xgc2_math::scene::SensorConfig& config);
  ~GpuMap();
  void scan(const xgc2_math::scene::SensorConfig&,const xgc2_math::Pose3&,double stamp,
            std::vector<std::uint8_t>& xyz);
private:
  struct State;
  std::unique_ptr<State> state_;
};
}
