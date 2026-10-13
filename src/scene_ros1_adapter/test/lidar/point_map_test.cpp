#include "xgc_scene/lidar/point_map.hpp"
#include <gtest/gtest.h>
#include <cstring>

namespace {
std::vector<std::uint8_t> cloud(std::initializer_list<float> values) {
  std::vector<float> points(values);
  std::vector<std::uint8_t> bytes(points.size() * sizeof(float));
  std::memcpy(bytes.data(), points.data(), bytes.size());
  return bytes;
}
TEST(PointMap, PenetrationAndOcclusionAreIndependentOfTheSavedCloud) {
  xgc_scene::lidar::PointMap map(cloud({2,0,0, 4,0,0, -3,0,0}));
  xgc_scene::lidar::PointMap::Scratch scratch;
  std::vector<std::uint8_t> points;
  xgc2_math::scene::SensorConfig config;
  config.range=5; config.h_fov_deg=90; config.v_fov_deg=60;
  config.h_res=90; config.v_res=60;
  config.mode=xgc2_math::scene::SensorConfig::kPenetrating;
  map.scan(config, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), scratch, points);
  EXPECT_EQ(points.size(), 24u);
  config.mode=xgc2_math::scene::SensorConfig::kRaycast;
  map.scan(config, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), scratch, points);
  ASSERT_EQ(points.size(), 12u);
  float x=0; std::memcpy(&x, points.data(), sizeof(x)); EXPECT_FLOAT_EQ(x,2);
  map.scan(config, Eigen::Vector3d::Zero(),
           Eigen::Quaterniond(Eigen::AngleAxisd(3.141592653589793, Eigen::Vector3d::UnitZ())),
           scratch, points);
  ASSERT_EQ(points.size(), 12u);
  std::memcpy(&x, points.data(), sizeof(x)); EXPECT_FLOAT_EQ(x,-3);
}
TEST(PointMap, RadiusQueriesVisitAllIndexedLeavesAndApplyTheCrop) {
  std::vector<float> values;
  for(int i=1;i<=256;++i) values.insert(values.end(),{float(i),0,0});
  std::vector<std::uint8_t> bytes(values.size()*sizeof(float));
  std::memcpy(bytes.data(),values.data(),bytes.size());
  xgc_scene::lidar::PointMap map(bytes);
  xgc_scene::lidar::PointMap::Scratch scratch;
  std::vector<std::uint8_t> result;
  xgc2_math::scene::SensorConfig config;
  config.mode=xgc2_math::scene::SensorConfig::kPenetrating;
  config.range=5; config.h_fov_deg=360; config.v_fov_deg=180;
  config.penetrating_heading_crop=true; config.heading_cos_min=0;
  config.vertical_slab_tan=1;
  map.scan(config,{128,0,0},Eigen::Quaterniond::Identity(),scratch,result);
  EXPECT_EQ(result.size()/12,5u);
}
}
