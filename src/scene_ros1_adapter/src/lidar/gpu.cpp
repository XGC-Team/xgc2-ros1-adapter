#include "xgc_scene/lidar/gpu.hpp"
#include <xgc2_sensing/loader.hpp>
#include <cstring>
#include <stdexcept>
namespace xgc_scene::lidar {
struct GpuMap::State {
  const xgc2_sensing_gpu_api* api;
  void* renderer = nullptr;
  std::vector<std::uint8_t> xyzi;
  ~State(){if(renderer) api->destroy(renderer);}
};
static xgc2_sensing_gpu_projection projection(const xgc2_math::scene::SensorConfig& c) {
  return {c.range,c.min_range,c.h_fov_deg,c.v_fov_deg,c.h_res,c.v_res,c.surface_spacing,10,
          uint32_t(c.mode==xgc2_math::scene::SensorConfig::kPenetrating),
          uint32_t(c.penetrating_heading_crop),c.heading_cos_min,c.vertical_slab_tan};
}
GpuMap::GpuMap(const std::shared_ptr<const xgc2_math::scene::LidarScene>& scene,
               const xgc2_math::scene::SensorConfig& config):state_(new State) {
  state_->api=&xgc2_sensing::gpuLibrary();
  std::vector<float> xyz;xyz.reserve(scene->sampleCount()*3);
  for(const auto& p:scene->samples()) for(int i=0;i<3;++i) xyz.push_back(static_cast<float>(p[i]));
  auto params=projection(config);char error[512]={};
  state_->renderer=state_->api->create(xyz.data(),scene->sampleCount(),&params,error,sizeof(error));
  if(!state_->renderer) throw std::runtime_error(error);
  state_->xyzi.reserve(scene->sampleCount()*16);
}
GpuMap::~GpuMap()=default;
void GpuMap::scan(const xgc2_math::scene::SensorConfig& config,const xgc2_math::Pose3& pose,
                  double stamp,std::vector<std::uint8_t>& xyz) {
  const auto params=projection(config);
  const double p[3]={pose.position.x(),pose.position.y(),pose.position.z()};
  const double q[4]={pose.orientation.w(),pose.orientation.x(),pose.orientation.y(),pose.orientation.z()};
  auto grow=[](void* ctx,size_t bytes)->void* { auto& out=*static_cast<std::vector<std::uint8_t>*>(ctx);try {out.resize(bytes);return out.data();}catch(...){return nullptr;} };
  char error[512]={};
  if(state_->api->scan(state_->renderer,p,q,stamp,&params,grow,&state_->xyzi,error,sizeof(error))) throw std::runtime_error(error);
  const auto count=state_->xyzi.size()/16;xyz.resize(count*12);
  for(size_t i=0;i<count;++i) std::memcpy(xyz.data()+i*12,state_->xyzi.data()+i*16,12);
}
}
