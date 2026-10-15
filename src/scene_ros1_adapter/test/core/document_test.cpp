#include "xgc_scene/document.hpp"
#include "xgc_scene/json_io.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <functional>
#include <limits>

namespace xgc_scene {
namespace {

void rejects(const std::function<void()>& call, const char* message) {
  try { call(); FAIL() << "Expected " << message; }
  catch (const SceneError& error) { EXPECT_STREQ(error.what(), message); }
}

Json box() {
  return parseJson(R"({"id":"wall","parts":[{"id":"body","geometry":{"type":"box","size":[1,2,3]}}]})");
}
Json tetrahedron() {
  return parseJson(R"({"type":"convex","vertices":[[0,0,0],[1,0,0],[0,1,0],[0,0,1]],"triangles":[0,1,2,0,3,1,0,2,3,1,3,2]})");
}

TEST(Document, PythonNumericTypesAndBounds) {
  for (const auto& value : {Json(true), Json("1"), Json(nullptr), Json(std::numeric_limits<double>::infinity()),
                           Json(std::numeric_limits<double>::quiet_NaN())})
    rejects([&] { number(value, "value"); }, "value must be finite");
  EXPECT_EQ(number(-1000000, "value"), -1000000.0);
  EXPECT_EQ(number(1000000, "value"), 1000000.0);
  rejects([] { number(1000001, "value"); }, "value is outside the supported range");
  rejects([] { geometry(parseJson("{\"type\":\"sphere\",\"radius\":" + std::string(1000, '9') + "}")); }, "radius must be finite");
  const auto normalized = vector(Json::array({1, 2, -0.0}), "position");
  for (const auto& value : normalized) EXPECT_TRUE(value.is_number_float());
  EXPECT_TRUE(std::signbit(normalized[2].get<double>()));
  rejects([] { vector(Json::array({1, 2}), "position"); }, "position must have 3 components");
  rejects([] { vector(Json::array({1, 2, false}), "position"); }, "position must be finite");
}

TEST(Document, FieldErrorsAndStableIdentifiers) {
  rejects([] { fields(Json::array(), {"id"}); }, "Expected an object");
  rejects([] { fields(Json{{"z", 1}, {"a", 2}}, {"id"}, {"id", "frame"}); },
          "Invalid fields: unknown=['a', 'z'], missing=['frame', 'id']");
  EXPECT_EQ(identifier(std::string(128, 'a')), std::string(128, 'a'));
  for (const auto& value : {Json(""), Json("_id"), Json("a/b"), Json("a\n"), Json(std::string(129, 'a')), Json(1)})
    rejects([&] { identifier(value); }, "ID must be a stable nonempty identifier");
  rejects([] { geometry(Json{{"type", nullptr}}); }, "Unsupported geometry None; no shape substitution is performed");
  rejects([] { motion(Json{{"type", true}}); }, "Unsupported motion True");
  rejects([] { geometry(Json{{"type", "it's"}}); }, "Unsupported geometry \"it's\"; no shape substitution is performed");
}

TEST(Document, ParametricRingKeepsNamedDimensionsAndRejectsHoleCollapse) {
  const auto valid = parseJson(R"({"type":"ring","radii":[3,1],"tubeRadius":0.8})");
  EXPECT_EQ(geometry(valid), valid);  // No curvature gate: 0.8 > 1/3.
  for (const auto& radii : {Json::array({1}), Json::array({1, 2, 3}), Json::array({true, 2}),
                            Json::array({0, 2}), Json::array({-1, 2}), Json::array({1e5, 2})}) {
    auto value = valid; value["radii"] = radii;
    EXPECT_THROW(geometry(value), SceneError);
  }
  for (const auto& tube : {Json(true), Json(0), Json(-1), Json(1), Json(2),
                           Json(std::numeric_limits<double>::infinity())}) {
    auto value = valid; value["tubeRadius"] = tube;
    EXPECT_THROW(geometry(value), SceneError);
  }
  for (const auto* key : {"radius", "height", "size", "segments", "params"}) {
    auto value = valid; value[key] = 1;
    EXPECT_THROW(geometry(value), SceneError);
  }
}

TEST(Document, PosePartAndNameNormalization) {
  EXPECT_EQ(pose(Json::object()), parseJson(R"({"position":[0.0,0.0,0.0],"orientation":[0.0,0.0,0.0,1.0]})"));
  EXPECT_EQ(pose(Json{{"orientation", Json::array({0, 0, 0, -2})}}).at("orientation")[3], -1.0);
  rejects([] { pose(Json{{"orientation", Json::array({0, 0, 0, 0})}}); }, "Orientation quaternion must be nonzero");
  auto value = box();
  value["parts"][0]["color"] = Json::array({0.2, 0.3, 0.4, 0.5});
  EXPECT_EQ(obstacle(value).at("parts")[0].at("color"), Json::array({1.0, 0.5, 0.1, 1.0}));
  value["parts"][0]["color"][0] = 1.1;
  rejects([&] { obstacle(value); }, "Color channels must be between 0 and 1");
  value = box();
  value["parts"].push_back(value["parts"][0]);
  rejects([&] { obstacle(value); }, "Duplicate part ID body");
  value = box();
  value["name"] = "\xc2\xa0\xe2\x80\x83";
  rejects([&] { obstacle(value); }, "Obstacle name must have 1 to 256 characters");
  std::string name;
  for (int i = 0; i < 256; ++i) name += "\xc3\xa9";
  value["name"] = name;
  EXPECT_EQ(obstacle(value).at("name"), name);
  value["name"] = name + "x";
  rejects([&] { obstacle(value); }, "Obstacle name must have 1 to 256 characters");
}

TEST(Document, ConvexShapeChecksKeepPythonErrorOrder) {
  const auto mesh = geometry(tetrahedron());
  EXPECT_EQ(mesh.at("triangles"), Json::array({0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3}));
  auto value = tetrahedron();
  value["vertices"][3] = value["vertices"][0];
  value["triangles"] = nullptr;
  rejects([&] { geometry(value); }, "Convex geometry has duplicate vertices");
  for (const auto& index : {Json(true), Json(1.0), Json(-1), Json(4), Json(std::numeric_limits<std::uint64_t>::max())}) {
    value = tetrahedron(); value["triangles"][0] = index;
    rejects([&] { geometry(value); }, "Convex triangle index is invalid");
  }
  value = tetrahedron(); value["vertices"].push_back(Json::array({0.1, 0.1, 0.1}));
  rejects([&] { geometry(value); }, "Every convex vertex must belong to its surface");
  value = tetrahedron(); value["triangles"] = Json::array();
  EXPECT_EQ(geometry(value).at("vertices"), value.at("vertices"));
  EXPECT_FALSE(geometry(value).contains("triangles"));
  value = tetrahedron(); value["triangles"] = std::vector<int>(24579, 0);
  rejects([&] { geometry(value); }, "Convex geometry requires triangle index triplets");
  value = tetrahedron(); value["vertices"] = std::vector<Json>(4097, Json::array({0, 0, 0}));
  rejects([&] { geometry(value); }, "Convex geometry requires 4 to 4096 vertices");
}

TEST(Document, MotionAndPartErrors) {
  rejects([] { motion(parseJson(R"({"type":"constant_twist","linear":[true,0,0]})")); }, "linear velocity must be finite");
  rejects([] { motion(parseJson(R"({"type":"constant_twist","angular":[0,0]})")); }, "angular velocity must have 3 components");
  rejects([] { motion(parseJson(R"({"type":"ping_pong","point_a":[0,0,0],"point_b":[1,0,0],"speed":0})")); }, "speed is outside the supported range");
  rejects([] { motion(parseJson(R"({"type":"circle","center":[0,0,0],"radius":0,"angular_speed":1})")); }, "circle radius is outside the supported range");
  auto value = box();
  value["parts"][0]["pose"] = Json{{"orientation", Json::array({0, 0, 0, 0})}};
  rejects([&] { obstacle(value); }, "Orientation quaternion must be nonzero");
  value = box(); value["parts"][0]["geometry"] = Json::object();
  rejects([&] { obstacle(value); }, "Unsupported geometry None; no shape substitution is performed");
}

TEST(JsonIo, DuplicateFieldsAndInsertionOrder) {
  rejects([] { parseJson(R"({"outer":[{"a":1,"a":2}]})"); }, "Duplicate scene field a");
  rejects([] { uniqueObject({{1, 2}}); }, "Scene field names must be strings");
  rejects([] { uniqueObject({{"a", 1}, {"a", 2}}); }, "Duplicate scene field a");
  auto value = parseJson(R"({"z":1,"a":2,"m":3})");
  value["a"] = 4; value.erase("m"); value["b"] = 5;
  EXPECT_EQ(dumpJson(value), "{\"z\": 1, \"a\": 4, \"b\": 5}");
  EXPECT_EQ(value, (Json{{"b", 5}, {"a", 4}, {"z", 1}}));
  rejects([] { dumpJson(parseJson("[NaN]")); }, "Out of range float values are not JSON compliant");
}

}  // namespace
}  // namespace xgc_scene
