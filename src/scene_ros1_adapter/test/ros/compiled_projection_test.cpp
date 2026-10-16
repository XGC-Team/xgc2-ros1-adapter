#include "xgc_scene/document.hpp"
#include "xgc_scene/ros_outputs.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <ros/serialization.h>
#include <xgc2_math/geometry/geometry_payload.hpp>

namespace xgc_scene {
namespace {

class CompiledOutputs final : public RosOutputs {
 public:
  void snapshot(const xgc2_geometry_msgs::SceneSnapshot& value) override { snapshots.push_back(value); }
  void state(const xgc2_geometry_msgs::SceneState& value) override { states.push_back(value); }
  void document(const std::string&) override {}
  void markers(const visualization_msgs::MarkerArray& value) override { markers_list.push_back(value); }
  void staticTransforms(const std::vector<geometry_msgs::TransformStamped>& value) override { fixed.push_back(value); }
  void dynamicTransforms(const std::vector<geometry_msgs::TransformStamped>& value) override { moving.push_back(value); }
  std::vector<xgc2_geometry_msgs::SceneSnapshot> snapshots;
  std::vector<xgc2_geometry_msgs::SceneState> states;
  std::vector<visualization_msgs::MarkerArray> markers_list;
  std::vector<std::vector<geometry_msgs::TransformStamped>> fixed, moving;
};

Json sharedSource(std::size_t count = 100) {
  Json obstacles = Json::array();
  for (std::size_t i = 0; i < count; ++i)
    obstacles.push_back(Json{{"id", "instance" + std::to_string(i)}, {"model", "shared"},
                             {"pose", Json{{"position", Json::array({static_cast<double>(i), 2.0, 3.0})}}}});
  return Json{{"schema", kSchema}, {"id", "compiled-projection"}, {"frame", "world"},
              {"models", Json{{"shared", Json{{"parts", Json::array({Json{{"id", "body"},
                  {"pose", Json{{"position", Json::array({0.0, 0.25, 0.0})}}},
                  {"geometry", Json{{"type", "box"}, {"size", Json::array({1.0, 2.0, 3.0})}}}}})}}}}},
              {"obstacles", std::move(obstacles)}};
}

TEST(CompiledSceneProjection, HundredInstancesEncodeOneDefinitionAndKeepModelLocalParts) {
  const auto source = sharedSource();
  const auto compiled = compile(source);
  ASSERT_EQ(compiled.at("geometries").size(), 1u);
  ASSERT_EQ(compiled.at("models").size(), 1u);
  for (const auto& item : compiled.at("obstacles")) EXPECT_FALSE(item.contains("parts"));
  SceneProjection projection("/compiled/");
  const auto snapshot = projection.snapshot(compiled, "epoch", 1, 10);
  ASSERT_EQ(snapshot.geometries.size(), 1u);
  ASSERT_EQ(snapshot.obstacles.size(), 100u);
  EXPECT_EQ(projection.counters().geometry_encodes, 1u);
  for (std::size_t i = 0; i < snapshot.obstacles.size(); ++i) {
    const auto& body = snapshot.obstacles[i];
    ASSERT_EQ(body.parts.size(), 1u);
    EXPECT_EQ(body.id, "instance" + std::to_string(i));
    EXPECT_DOUBLE_EQ(body.pose.position.x, static_cast<double>(i));
    EXPECT_DOUBLE_EQ(body.parts[0].pose.position.y, .25);
    EXPECT_EQ(body.parts[0].geometry_index, 0u);
    EXPECT_FLOAT_EQ(body.parts[0].color.r, 1.0f);
    EXPECT_EQ(body.motion_type, "hold");
    EXPECT_FALSE(body.dynamic);
  }
  EXPECT_TRUE(source.at("models").at("shared").at("parts")[0].contains("geometry"));
  EXPECT_FALSE(compiled.at("models").at("shared").at("parts")[0].contains("geometry"));
}

TEST(CompiledSceneProjection, DefinitionAndModelEditsInvalidateDependentMarkersWithoutChangingOldSnapshots) {
  auto compiled = compile(sharedSource());
  SceneProjection projection("/compiled/");
  CompiledOutputs outputs;
  const auto first = projection.snapshot(compiled, "epoch", 1, 10);
  projection.definition(compiled, "epoch", 1, first, outputs);
  const auto initial_markers = projection.counters().part_markers;
  EXPECT_EQ(initial_markers, 100u);
  compiled["geometries"][0]["size"][0] = 4.0;
  const auto changed = projection.snapshot(compiled, "epoch", 2, 11);
  projection.definition(compiled, "epoch", 2, changed, outputs);
  EXPECT_EQ(projection.counters().part_markers - initial_markers, 100u);
  for (std::size_t i = 1; i < outputs.markers_list.back().markers.size(); ++i)
    EXPECT_DOUBLE_EQ(outputs.markers_list.back().markers[i].scale.x, 4.0);
  const auto original = xgc2_math::geometry::decodeGeometry({first.geometries[0].type, first.geometries[0].parameters});
  EXPECT_DOUBLE_EQ(original.size[0], 1.0);
  EXPECT_EQ(first.obstacles[0].parts[0].geometry_index, 0u);
  compiled["models"]["shared"]["parts"][0]["pose"]["position"][1] = .75;
  compiled["models"]["shared"]["parts"][0]["color"] = Json::array({0.0, 0.0, 1.0, 1.0});
  const auto recolored = projection.snapshot(compiled, "epoch", 3, 12);
  projection.definition(compiled, "epoch", 3, recolored, outputs);
  for (const auto& body : recolored.obstacles) {
    EXPECT_DOUBLE_EQ(body.parts[0].pose.position.y, .75);
    EXPECT_FLOAT_EQ(body.parts[0].color.b, 1.0f);
    EXPECT_FLOAT_EQ(body.parts[0].color.r, 0.0f);
  }
  EXPECT_DOUBLE_EQ(first.obstacles[0].parts[0].pose.position.y, .25);
  EXPECT_FLOAT_EQ(first.obstacles[0].parts[0].color.r, 1.0f);
  const auto stable = projection.counters();
  const auto same = projection.snapshot(compiled, "epoch", 4, 13);
  projection.definition(compiled, "epoch", 4, same, outputs);
  EXPECT_EQ(projection.counters().part_markers, stable.part_markers);
  EXPECT_EQ(projection.counters().obstacle_messages, stable.obstacle_messages);
}

TEST(CompiledSceneProjection, IndexedMeshDefinitionAppearsOnceForHundredInstancesOnTheRealWire) {
  auto source = sharedSource();
  Json vertices = Json::array(), triangles = Json::array();
  for (std::uint32_t i = 0; i < 128; ++i) {
    const auto first = static_cast<std::uint32_t>(vertices.size());
    const double x = i * 3.0;
    vertices.push_back(Json::array({x, 0.0, 0.0}));
    vertices.push_back(Json::array({x + 1, 0.0, 0.0}));
    vertices.push_back(Json::array({x, 1.0, 0.0}));
    vertices.push_back(Json::array({x, 0.0, 1.0}));
    for (const auto index : {0u, 2u, 1u, 0u, 1u, 3u, 0u, 3u, 2u, 1u, 2u, 3u})
      triangles.push_back(first + index);
  }
  source["models"]["shared"]["parts"][0]["geometry"] =
      Json{{"type", "mesh"}, {"vertices", std::move(vertices)}, {"triangles", std::move(triangles)}};
  const auto compiled = compile(source);
  ASSERT_EQ(compiled.at("geometries").size(), 1u);
  ASSERT_EQ(compiled.at("models").size(), 1u);
  SceneProjection projection("/compiled-mesh/");
  const auto snapshot = projection.snapshot(compiled, "epoch", 1, 0);
  ASSERT_EQ(snapshot.geometries.size(), 1u);
  EXPECT_EQ(snapshot.geometries[0].type, 9u);
  EXPECT_EQ(projection.counters().geometry_encodes, 1u);
  auto one = snapshot;
  one.obstacles.resize(1);
  std::size_t added = 0;
  for (std::size_t i = 1; i < snapshot.obstacles.size(); ++i)
    added += ros::serialization::serializationLength(snapshot.obstacles[i]);
  EXPECT_EQ(ros::serialization::serializationLength(snapshot) -
                ros::serialization::serializationLength(one), added);
  std::vector<std::uint8_t> bytes(ros::serialization::serializationLength(snapshot));
  ros::serialization::OStream output(bytes.data(), bytes.size());
  ros::serialization::serialize(output, snapshot);
  xgc2_geometry_msgs::SceneSnapshot decoded;
  ros::serialization::IStream input(bytes.data(), bytes.size());
  ros::serialization::deserialize(input, decoded);
  ASSERT_EQ(decoded.geometries.size(), 1u);
  EXPECT_EQ(decoded.geometries[0].parameters, snapshot.geometries[0].parameters);
  ASSERT_EQ(decoded.obstacles.size(), 100u);
  for (const auto& item : decoded.obstacles) EXPECT_EQ(item.parts[0].geometry_index, 0u);
}

TEST(CompiledSceneProjection, TableOrderIsAuthoritativeAndPartReferencesDoNotBecomeInline) {
  auto compiled = compile(sharedSource(1));
  compiled["geometries"].push_back(Json{{"type", "sphere"}, {"radius", .4}});
  compiled["models"]["shared"]["parts"][0]["geometryIndex"] = 1;
  SceneProjection projection("/compiled/");
  const auto snapshot = projection.snapshot(compiled, "epoch", 1, 0);
  ASSERT_EQ(snapshot.geometries.size(), 2u);
  EXPECT_EQ(snapshot.geometries[0].type, 1u);
  EXPECT_EQ(snapshot.geometries[1].type, 2u);
  EXPECT_EQ(snapshot.obstacles[0].parts[0].geometry_index, 1u);
  EXPECT_EQ(projection.counters().geometry_encodes, 2u);
}

TEST(CompiledSceneProjection, StaticTicksDoNoDefinitionOrRosterWork) {
  const auto compiled = compile(sharedSource());
  SceneProjection projection("/compiled/");
  CompiledOutputs outputs;
  const auto snapshot = projection.snapshot(compiled, "epoch", 1, 0);
  projection.definition(compiled, "epoch", 1, snapshot, outputs);
  const auto before = projection.counters();
  for (int i = 0; i < 100; ++i) projection.tick(i * .1, i * .1, true, outputs);
  const auto after = projection.counters();
  EXPECT_EQ(after.frame_scans, before.frame_scans);
  EXPECT_EQ(after.snapshot_scans, before.snapshot_scans);
  EXPECT_EQ(after.definition_scans, before.definition_scans);
  EXPECT_EQ(after.geometry_encodes, before.geometry_encodes);
  EXPECT_EQ(after.obstacle_messages, before.obstacle_messages);
  EXPECT_EQ(after.part_markers, before.part_markers);
  EXPECT_EQ(after.moving_states, before.moving_states);
  EXPECT_EQ(after.hold_states, before.hold_states);
  EXPECT_EQ(after.state_scans - before.state_scans, 100u); // Existing empty heartbeat only.
  EXPECT_TRUE(outputs.moving.empty());
  ASSERT_EQ(outputs.states.size(), 100u);
  for (const auto& value : outputs.states) EXPECT_TRUE(value.obstacles.empty());
}

TEST(CompiledSceneProjection, ExistingMotionUsesTheExistingEvaluatorAndOnlyDynamicSlots) {
  auto source = sharedSource(4);
  source["obstacles"][1]["motion"] = Json{{"type", "constant_twist"}, {"linear", Json::array({2, 0, 0})}};
  source["obstacles"][2]["motion"] = Json{{"type", "ping_pong"}, {"point_a", Json::array({0, 0, 0})},
                                       {"point_b", Json::array({4, 0, 0})}, {"speed", 1}};
  source["obstacles"][2]["pose"]["position"] = Json::array({0, 0, 0});
  source["obstacles"][3]["motion"] = Json{{"type", "circle"}, {"center", Json::array({0, 0, 0})},
                                       {"radius", 2}, {"angular_speed", 1}};
  source["obstacles"][3]["pose"]["position"] = Json::array({2, 0, 0});
  const auto compiled = compile(source);
  SceneProjection projection("/compiled/");
  CompiledOutputs outputs;
  const auto snapshot = projection.snapshot(compiled, "epoch", 1, 0);
  projection.definition(compiled, "epoch", 1, snapshot, outputs);
  projection.tick(1, 1, true, outputs);
  const auto& states = outputs.states.back().obstacles;
  ASSERT_EQ(states.size(), 3u);
  EXPECT_EQ(states[0].id, "instance1");
  EXPECT_DOUBLE_EQ(states[0].pose.position.x, 3);
  EXPECT_DOUBLE_EQ(states[0].twist.linear.x, 2);
  EXPECT_EQ(states[1].id, "instance2");
  EXPECT_DOUBLE_EQ(states[1].pose.position.x, 1);
  EXPECT_EQ(states[2].id, "instance3");
  EXPECT_NEAR(states[2].pose.position.x, 2 * std::cos(1.0), 1e-12);
  EXPECT_EQ(projection.counters().moving_states, 3u);
  EXPECT_EQ(projection.counters().hold_states, 0u);
}

TEST(CompiledSceneProjection, RejectsAuthorInlineUnknownReferencesAndUnsupportedMotion) {
  SceneProjection projection("/compiled/");
  EXPECT_THROW(projection.snapshot(sharedSource(1), "epoch", 1, 0), SceneError);
  auto compiled = compile(sharedSource(1));
  compiled["models"]["shared"]["parts"][0]["geometryIndex"] = 1;
  EXPECT_THROW(projection.snapshot(compiled, "epoch", 1, 0), SceneError);
  compiled["models"]["shared"]["parts"][0]["geometryIndex"] = -1;
  EXPECT_THROW(projection.snapshot(compiled, "epoch", 1, 0), SceneError);
  compiled = compile(sharedSource(1));
  compiled["obstacles"][0]["model"] = "missing";
  EXPECT_THROW(projection.snapshot(compiled, "epoch", 1, 0), SceneError);
  compiled = compile(sharedSource(1));
  compiled["obstacles"][0]["parts"] = Json::array();
  EXPECT_THROW(projection.snapshot(compiled, "epoch", 1, 0), SceneError);
  compiled = compile(sharedSource(1));
  compiled["obstacles"][0]["motion"]["type"] = "waypoints";
  EXPECT_THROW(projection.snapshot(compiled, "epoch", 1, 0), SceneError);
}

TEST(CompiledSceneProjection, ExplicitGroundModesUseTheSameTableWithoutBusinessObstacleIds) {
  auto source = sharedSource(1);
  source["ground"] = Json{{"mode", "model"}, {"model", "shared"},
                          {"pose", Json{{"position", Json::array({2.0, 3.0, -1.0})}}}};
  SceneProjection projection("/ground/");
  const auto model = projection.snapshot(compile(source), "epoch", 1, 0);
  EXPECT_EQ(model.ground.mode, xgc2_geometry_msgs::SceneGround::MODE_MODEL);
  EXPECT_DOUBLE_EQ(model.ground.pose.position.z, -1);
  ASSERT_EQ(model.ground.parts.size(), 1u);
  ASSERT_EQ(model.geometries.size(), 1u);
  ASSERT_EQ(model.obstacles.size(), 1u);
  EXPECT_EQ(model.obstacles[0].id, "instance0");
  EXPECT_EQ(model.ground.parts[0].geometry_index, model.obstacles[0].parts[0].geometry_index);
  EXPECT_DOUBLE_EQ(model.ground.parts[0].pose.position.y, .25);
  source["ground"] = Json{{"mode", "default_plane"}, {"z", 1.25}};
  const auto plane = projection.snapshot(compile(source), "epoch", 2, 0);
  EXPECT_EQ(plane.ground.mode, xgc2_geometry_msgs::SceneGround::MODE_DEFAULT_PLANE);
  EXPECT_DOUBLE_EQ(plane.ground.z, 1.25);
  EXPECT_TRUE(plane.ground.parts.empty());
  source["ground"] = Json{{"mode", "none"}};
  const auto none = projection.snapshot(compile(source), "epoch", 3, 0);
  EXPECT_EQ(none.ground.mode, xgc2_geometry_msgs::SceneGround::MODE_NONE);
  EXPECT_DOUBLE_EQ(none.ground.z, 0);
  EXPECT_TRUE(none.ground.parts.empty());
  EXPECT_EQ(none.obstacles.size(), 1u);
}

TEST(CompiledSceneProjection, GroundModelMarkersAndFrameAreColdAndDoNoWorkInHundredStaticTicks) {
  auto source = sharedSource(0);
  source["ground"] = Json{{"mode", "model"}, {"model", "shared"},
                          {"pose", Json{{"position", Json::array({2.0, 3.0, -1.0})}}}};
  auto compiled = compile(source);
  SceneProjection projection("/ground/");
  CompiledOutputs outputs;
  const auto snapshot = projection.snapshot(compiled, "epoch", 1, 0);
  projection.definition(compiled, "epoch", 1, snapshot, outputs);
  ASSERT_TRUE(snapshot.obstacles.empty());
  ASSERT_EQ(outputs.fixed.size(), 1u);
  ASSERT_EQ(outputs.fixed.back().size(), 1u);
  EXPECT_DOUBLE_EQ(outputs.fixed.back()[0].transform.translation.z, -1);
  ASSERT_EQ(outputs.markers_list.back().markers.size(), 2u); // DELETEALL + ground part.
  EXPECT_EQ(outputs.markers_list.back().markers[1].header.frame_id,
            outputs.fixed.back()[0].child_frame_id);
  EXPECT_DOUBLE_EQ(outputs.markers_list.back().markers[1].pose.position.y, .25);
  const auto before = projection.counters();
  for (int i = 0; i < 100; ++i) projection.tick(i * .1, i * .1, true, outputs);
  const auto after = projection.counters();
  EXPECT_EQ(after.frame_scans, before.frame_scans);
  EXPECT_EQ(after.snapshot_scans, before.snapshot_scans);
  EXPECT_EQ(after.definition_scans, before.definition_scans);
  EXPECT_EQ(after.geometry_encodes, before.geometry_encodes);
  EXPECT_EQ(after.obstacle_messages, before.obstacle_messages);
  EXPECT_EQ(after.part_markers, before.part_markers);
  EXPECT_EQ(after.hold_states, before.hold_states);
  EXPECT_EQ(after.moving_states, before.moving_states);
  EXPECT_EQ(outputs.fixed.size(), 1u);
  EXPECT_EQ(outputs.markers_list.size(), 1u);
  EXPECT_TRUE(outputs.moving.empty());
  for (const auto& value : outputs.states) EXPECT_TRUE(value.obstacles.empty());
  compiled["geometries"][0]["size"][0] = 4.0;
  const auto edited = projection.snapshot(compiled, "epoch", 2, 1);
  projection.definition(compiled, "epoch", 2, edited, outputs);
  EXPECT_DOUBLE_EQ(outputs.markers_list.back().markers[1].scale.x, 4);
  EXPECT_EQ(projection.counters().part_markers - after.part_markers, 1u);
}

TEST(CompiledSceneProjection, DefaultPlaneAndNonePublishNoSyntheticGroundMarkerOrFrame) {
  for (const auto* mode : {"default_plane", "none"}) {
    auto source = sharedSource(0);
    source["ground"] = Json{{"mode", mode}};
    const auto compiled = compile(source);
    SceneProjection projection("/ground/");
    CompiledOutputs outputs;
    const auto snapshot = projection.snapshot(compiled, "epoch", 1, 0);
    projection.definition(compiled, "epoch", 1, snapshot, outputs);
    EXPECT_TRUE(snapshot.ground.parts.empty());
    EXPECT_TRUE(snapshot.obstacles.empty());
    EXPECT_TRUE(outputs.fixed.empty());
    ASSERT_EQ(outputs.markers_list.back().markers.size(), 1u);
    EXPECT_EQ(outputs.markers_list.back().markers[0].action, visualization_msgs::Marker::DELETEALL);
  }
}

TEST(CompiledSceneProjection, UnknownGroundModesAndModelReferencesAreExplicitlyRefused) {
  SceneProjection projection("/ground/");
  auto compiled = compile(sharedSource(0));
  for (const auto* mode : {"", "terrain"}) {
    compiled["ground"] = Json{{"mode", mode}};
    EXPECT_THROW(projection.snapshot(compiled, "epoch", 1, 0), SceneError);
  }
  compiled["ground"] = Json{{"mode", "model"}, {"model", "absent"}, {"pose", pose(Json::object())}};
  EXPECT_THROW(projection.snapshot(compiled, "epoch", 1, 0), SceneError);
}

}  // namespace
}  // namespace xgc_scene
