#include "../geometry_wire.hpp"
#include "xgc_scene/lidar_ros/scene_tap.hpp"
#include <gtest/gtest.h>

namespace xgc_scene::lidar_ros {
namespace {
struct Outputs : RosOutputs {
  void snapshot(const xgc2_geometry_msgs::SceneSnapshot&) override {}
  void state(const xgc2_geometry_msgs::SceneState&) override {}
  void document(const std::string&) override {}
  void markers(const visualization_msgs::MarkerArray&) override {}
  void staticTransforms(const std::vector<geometry_msgs::TransformStamped>&) override {}
  void dynamicTransforms(const std::vector<geometry_msgs::TransformStamped>&) override {}
};
struct Sink : lidar::Sink {
  void onPoints(std::size_t, lidar::CloudView) override {}
  void onBeams(std::size_t, lidar::CloudView) override {}
  void onReference(lidar::CloudView, lidar::ReferenceAction) override {}
};
TEST(LidarRingMapping, NamedParametersBothPosesDynamicStateAndRejectedInvalidGeometry) {
  Outputs outputs;
  Sink sink;
  auto manifest = lidar::parseManifest(R"({"schemaVersion":1,"robots":[{"namespace":"/A","hRes":1,"vRes":1}],"referenceCloud":{"surfaceSpacing":0.2}})");
  lidar::Engine engine(manifest, sink);
  SceneTap tap(outputs, &engine, [] { return 1000000000; });
  xgc2_geometry_msgs::SceneSnapshot definition;
  definition.header.frame_id = "world";
  definition.epoch = "ring";
  definition.revision = 1;
  xgc2_geometry_msgs::SceneObstacle obstacle;
  obstacle.id = "elliptic";
  obstacle.dynamic = true;
  obstacle.pose.position.x = 3;
  const Eigen::Quaterniond parent(Eigen::AngleAxisd(.6, Eigen::Vector3d::UnitY()));
  const Eigen::Quaterniond child(Eigen::AngleAxisd(.4, Eigen::Vector3d::UnitZ()));
  const auto assign=[](geometry_msgs::Quaternion& q, const Eigen::Quaterniond& r) { q.x=r.x();q.y=r.y();q.z=r.z();q.w=r.w(); };
  assign(obstacle.pose.orientation, parent);
  xgc2_geometry_msgs::ScenePart part;
  part.pose.position.y = .3;
  assign(part.pose.orientation, child);
  part.geometry = geometry_test::wire(geometry_test::Geometry::ring({1, 1.2}, .05));
  obstacle.parts.push_back(part);
  definition.obstacles.push_back(obstacle);
  tap.application("ring", 1, true);
  tap.snapshot(definition);
  EXPECT_FALSE(engine.sceneReady());
  xgc2_geometry_msgs::SceneState state;
  state.header.frame_id = "world";state.epoch="ring";state.revision=1;
  xgc2_geometry_msgs::SceneObstacleState moving;
  moving.id=obstacle.id;moving.pose=obstacle.pose;state.obstacles.push_back(moving);
  tap.state(state);
  ASSERT_TRUE(engine.sceneReady()) << tap.error();
  const Eigen::Vector3d direction=parent*child*Eigen::Vector3d::UnitX();
  Eigen::Vector3d center=Eigen::Vector3d(3,0,0)+parent*Eigen::Vector3d(0,.3,0);
  EXPECT_NEAR(engine.sensorScene(0)->closest(center, direction, 10),.95,1e-10);
  EXPECT_TRUE(std::isinf(engine.sensorScene(0)->closest(center,parent*child*Eigen::Vector3d::UnitZ(),10)));
  state.obstacles[0].pose.position.x += 2;
  tap.state(state);
  center.x() += 2;
  ASSERT_TRUE(engine.sceneReady());
  EXPECT_NEAR(engine.sensorScene(0)->closest(center,direction,10),.95,1e-10);
  definition.revision=2;definition.obstacles[0].dynamic=false;
  definition.obstacles[0].parts[0].geometry.parameters.clear();
  tap.application("ring",2,true);
  tap.snapshot(definition);
  EXPECT_FALSE(engine.sceneReady());
  EXPECT_NE(tap.error().find("parameters"),std::string::npos);
}
} // namespace
} // namespace xgc_scene::lidar_ros
