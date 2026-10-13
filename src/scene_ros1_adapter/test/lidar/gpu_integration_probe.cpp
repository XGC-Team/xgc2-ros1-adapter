#include "xgc_scene/lidar/engine.hpp"
#include <cassert>
#include <atomic>
#include <chrono>
#include <algorithm>
#include <cstring>
#include <iostream>
using namespace xgc_scene::lidar;
using namespace xgc2_math::scene;
struct Probe : Sink {
  bool through=false;std::atomic<size_t> deliveries{0};
  void onPoints(size_t,CloudView c) override {
    assert(c.count>0 && c.point_step==12);size_t behind=0;
    for(size_t n=0;n<c.count;++n){float x;std::memcpy(&x,c.data+n*12,4);if(x>7)++behind;}
    assert((behind>0)==through);++deliveries;
  }
  void onBeams(size_t,CloudView) override {assert(false);}
  void onReference(CloudView,ReferenceAction) override {}
};
int main(int argc, char** argv) {
  const int count=argc>2?std::stoi(argv[2]):20;
  const int rounds=argc>2?12:3;
  std::vector<Obstacle> scene={Obstacle::box({5,0,0},{1,8,8}),Obstacle::box({9,0,0},{1,2,2})};
  if(argc>2)for(int i=0;i<100;++i)scene.push_back(Obstacle::box({15.+(i/10)*3.,(i%10-5)*3.,0},{1,2,3}));
  for(const std::string backend:{"cpu","gpu","auto"}) for(bool through:{false,true}) {
    if(argc>=2 && backend!=argv[1]) continue;
    FleetManifest fleet;Probe sink;sink.through=through;
    for(int n=0;n<count;++n){SensorSpec sensor;sensor.name_space="/uav"+std::to_string(n+1);sensor.backend=backend;
      sensor.config.mode=through?SensorConfig::kPenetrating:SensorConfig::kRaycast;
      sensor.config.range=20;sensor.config.min_range=.2;sensor.config.h_fov_deg=sensor.config.v_fov_deg=60;
      sensor.config.h_res=sensor.config.v_res=120;sensor.config.surface_spacing=n==19?.2:.1;
      fleet.robots.push_back(sensor);}
    Engine engine(std::move(fleet),sink);
    if(argc>3 && std::string(argv[3])=="fixed") {
      LidarScene sampled(scene,.1,true);std::vector<uint8_t> bytes(sampled.sampleCount()*12);
      for(size_t i=0;i<sampled.sampleCount();++i){float xyz[3];for(int a=0;a<3;++a)xyz[a]=sampled.samples()[i][a];std::memcpy(bytes.data()+i*12,xyz,12);}
      engine.setPointCloud(bytes,1);
    } else engine.setScene(scene,1);
    double steady_ms=0,worst_ms=0;
    for(int round=0;round<rounds;++round){const int64_t stamp=1000000000LL+round*200000000LL;
      for(int n=0;n<count;++n){xgc2_math::Pose3 pose;pose.position.x()=round>=2?.01*round:0;engine.setPose(n,pose,stamp);engine.setDemand(n,true);}
      const auto begin=std::chrono::steady_clock::now();
      assert(engine.tick(stamp)==static_cast<size_t>(count));
      const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
      if(round>=2){steady_ms+=ms;worst_ms=std::max(worst_ms,ms);}
    }
    assert(sink.deliveries==static_cast<size_t>(count*rounds));
    std::cout<<"PASS scene adapter "<<backend<<" through="<<through<<" robots="<<count<<" obstacles="<<scene.size()<<" steady_mean_ms="<<steady_ms/(rounds-2)<<" worst_ms="<<worst_ms<<"\n";
  }
}
