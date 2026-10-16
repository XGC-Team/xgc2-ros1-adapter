#include "xgc_scene/document.hpp"
#include "xgc_scene/json_io.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <fstream>

namespace xgc_scene {
namespace {

namespace fs = std::filesystem;

class Asset : public testing::Test {
 protected:
  void SetUp() override { root = fs::temp_directory_path() / "xgc-compiled-asset"; fs::remove_all(root); fs::create_directories(root); }
  void TearDown() override { fs::remove_all(root); }
  void write(const fs::path& path, const std::string& bytes) {
    fs::create_directories(root / path.parent_path());
    std::ofstream output(root / path);
    output << bytes;
  }
  fs::path root;
};

const char* kRing = R"(<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <library_geometries>
    <geometry id="ring"><mesh>
      <source id="positions"><float_array id="positions-array" count="24">-1 -1 0 1 -1 0 1 1 0 -1 1 0 -0.5 -0.5 0 0.5 -0.5 0 0.5 0.5 0 -0.5 0.5 0</float_array>
        <technique_common><accessor source="#positions-array" count="8" stride="3">
          <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
        </accessor></technique_common>
      </source>
      <vertices id="vertices"><input semantic="POSITION" source="#positions"/></vertices>
      <triangles count="8"><input semantic="VERTEX" source="#vertices" offset="0"/>
        <p>0 4 1 1 4 5 1 5 2 2 5 6 2 6 3 3 6 7 3 7 0 0 7 4</p>
      </triangles>
    </mesh></geometry>
  </library_geometries>
  <library_visual_scenes><visual_scene id="scene"><node id="ring">
    <translate>0 0 0.5</translate><scale>2 3 4</scale>
    <instance_geometry url="#ring"/>
  </node></visual_scene></library_visual_scenes>
  <scene><instance_visual_scene url="#scene"/></scene>
</COLLADA>
)";

Json scene(const std::string& body) {
  return parseJson(std::string("{\"schema\":\"xgc2.scene.v1\",\"id\":\"yard\",\"frame\":\"world\",") + body + "}");
}

TEST(Compiled, InlineScenesShareGeometriesAndStayFlat) {
  const auto source = scene("\"obstacles\":[{\"id\":\"a\",\"parts\":[{\"id\":\"body\",\"geometry\":{\"type\":\"box\",\"size\":[1,2,3]}}]},"
                            "{\"id\":\"b\",\"parts\":[{\"id\":\"body\",\"geometry\":{\"type\":\"box\",\"size\":[1,2,3]}}]}]");
  const auto authored = document(source);
  EXPECT_FALSE(authored.contains("models"));
  EXPECT_FALSE(authored.contains("ground"));
  EXPECT_EQ(authored.at("obstacles")[0].at("parts")[0].at("geometry").at("type"), "box");
  std::size_t reads = 99;
  const auto compiled = compile(source, {}, &reads);
  EXPECT_EQ(reads, 0u);
  EXPECT_EQ(compiled.at("schema"), kCompiledSchema);
  EXPECT_EQ(compiled.at("ground"), parseJson(R"({"mode":"default_plane","z":0.0})"));
  EXPECT_EQ(compiled.at("geometries").size(), 1u);
  EXPECT_FALSE(compiled.at("geometries")[0].contains("triangles"));
  EXPECT_EQ(compiled.at("models").at("__inline_a").at("parts")[0].at("geometryIndex"), 0);
  EXPECT_EQ(compiled.at("models").at("__inline_b").at("parts")[0].at("geometryIndex"), 0);
  EXPECT_EQ(compiled.at("obstacles")[0].at("model"), "__inline_a");
  EXPECT_FALSE(compiled.at("obstacles")[0].contains("parts"));
  EXPECT_EQ(authored.at("obstacles")[0].at("parts")[0].at("geometry").at("type"), "box");
}

TEST(Compiled, GroundAndModelReference) {
  const auto source = scene("\"ground\":{\"mode\":\"model\",\"model\":\"deck\",\"pose\":{\"position\":[0,0,-1]}},\"models\":{\"deck\":{\"parts\":[{\"id\":\"slab\",\"geometry\":{\"type\":\"box\",\"size\":[4,4,0.2]}}]}},\"obstacles\":[{\"id\":\"wall\",\"model\":\"deck\"}]");
  const auto compiled = compile(source);
  EXPECT_EQ(compiled.at("ground").at("mode"), "model");
  EXPECT_DOUBLE_EQ(compiled.at("ground").at("pose").at("position")[2].get<double>(), -1);
  EXPECT_EQ(compiled.at("obstacles")[0].at("model"), "deck");
  EXPECT_EQ(compiled.at("models").size(), 1u);
  const auto none = compile(scene("\"ground\":{\"mode\":\"none\"},\"obstacles\":[]"));
  EXPECT_EQ(none.at("ground"), parseJson(R"({"mode":"none"})"));
  EXPECT_THROW(compile(scene("\"ground\":{\"mode\":\"terrain\"},\"obstacles\":[]")), SceneError);
  EXPECT_THROW(document(scene("\"obstacles\":[{\"id\":\"ground_plane\",\"parts\":[{\"id\":\"body\",\"geometry\":{\"type\":\"box\",\"size\":[1,1,1]}}]}]")), SceneError);
  EXPECT_THROW(document(scene("\"obstacles\":[{\"id\":\"wall\",\"model\":\"missing\",\"parts\":[{\"id\":\"body\",\"geometry\":{\"type\":\"box\",\"size\":[1,1,1]}}]}]")), SceneError);
}

TEST_F(Asset, ResourceBakesScaleOnceAndKeepsTheHole) {
  write("ring.dae", kRing);
  write("barrier/model.sdf", R"(<?xml version="1.0"?>
<sdf version="1.6"><model name="barrier"><link name="body">
  <pose>1 2 3 0 0 0</pose>
  <collision name="hull"><pose>0 0 0.5 0 0 0</pose><geometry><box><size>0.8 0.2 1</size></box></geometry></collision>
  <collision name="shell"><geometry><mesh><uri>../ring.dae</uri><scale>1 1 1</scale></mesh></geometry></collision>
  <collision name="again"><geometry><mesh><uri>../ring.dae</uri><scale>1 1 1</scale></mesh></geometry></collision>
</link></model></sdf>
)");
  const auto source = scene("\"models\":{\"barrier\":{\"resource\":{\"format\":\"sdformat-model\",\"file\":\"barrier/model.sdf\"}}},\"obstacles\":[{\"id\":\"one\",\"model\":\"barrier\"}]");
  std::size_t reads = 0;
  const auto compiled = compile(source, root.string(), &reads);
  EXPECT_EQ(reads, 2u);
  const auto& parts = compiled.at("models").at("barrier").at("parts");
  EXPECT_EQ(parts.size(), 3u);
  EXPECT_DOUBLE_EQ(parts[0].at("pose").at("position")[2].get<double>(), 3.5);
  const auto& mesh = compiled.at("geometries").at(parts[1].at("geometryIndex").get<std::size_t>());
  EXPECT_EQ(mesh.at("type"), "mesh");
  EXPECT_EQ(mesh.at("triangles").size(), 24u);
  bool scaled = false;
  for (const auto& vertex : mesh.at("vertices")) {
    if (vertex.at(0).get<double>() == 2 && vertex.at(1).get<double>() == -3) scaled = true;
  }
  EXPECT_TRUE(scaled);
  EXPECT_EQ(parts[1].at("geometryIndex"), parts[2].at("geometryIndex"));
  EXPECT_EQ(compiled.at("models").at("barrier").at("resource").at("file"), (root / "barrier/model.sdf").lexically_normal().string());
  EXPECT_EQ(document(source).at("models").at("barrier").at("resource").at("file"), "barrier/model.sdf");
}

bool nearVertex(const Json& vertices, double x, double y, double z) {
  for (const auto& vertex : vertices) {
    if (std::abs(vertex.at(0).get<double>() - x) < 1e-4 && std::abs(vertex.at(1).get<double>() - y) < 1e-4 &&
        std::abs(vertex.at(2).get<double>() - z) < 1e-4) return true;
  }
  return false;
}

TEST(Compiled, ReviewAffineScaleAndFrames) {
  const fs::path assets = "/tmp/xgc-resource-compiler-review-20261017/assets";
  const auto load = [&](const std::string& file, const char* format) {
    return scene(std::string("\"models\":{\"resource\":{\"resource\":{\"format\":\"") + format +
                 "\",\"file\":\"" + file + "\"}}},\"obstacles\":[{\"id\":\"instance0\",\"model\":\"resource\",\"pose\":{\"position\":[100,200,300]}}]");
  };
  std::size_t reads = 0;
  const auto nested = compile(load("nested.gltf", "gltf"), assets.string(), &reads);
  EXPECT_EQ(reads, 1u);
  const auto& sheared = nested.at("geometries")[0].at("vertices");
  EXPECT_TRUE(nearVertex(sheared, 9, 17, 25));
  EXPECT_TRUE(nearVertex(sheared, 7.58578634262085, 19.121320724487305, 25));
  EXPECT_DOUBLE_EQ(nested.at("models").at("resource").at("parts")[0].at("pose").at("position")[0].get<double>(), 0);
  EXPECT_DOUBLE_EQ(nested.at("obstacles")[0].at("pose").at("position")[0].get<double>(), 100);

  reads = 0;
  const auto scaled = compile(load("scale.sdf", "sdformat-model"), assets.string(), &reads);
  EXPECT_EQ(reads, 2u);
  EXPECT_TRUE(nearVertex(scaled.at("geometries")[0].at("vertices"), 2, 6, 12));
  EXPECT_DOUBLE_EQ(scaled.at("models").at("resource").at("parts")[0].at("pose").at("position")[0].get<double>(), 0);

  const auto nested_model = compile(load("nested-model.sdf", "sdformat-model"), assets.string());
  ASSERT_EQ(nested_model.at("models").at("resource").at("parts").size(), 2u);
  bool at_origin = false, at_child = false;
  for (const auto& part : nested_model.at("models").at("resource").at("parts")) {
    const double x = part.at("pose").at("position")[0].get<double>();
    at_origin = at_origin || std::abs(x) < 1e-9;
    at_child = at_child || std::abs(x - 5) < 1e-9;
  }
  EXPECT_TRUE(at_origin && at_child);

  const auto relative = compile(load("relative-frame.sdf", "sdformat-model"), assets.string());
  EXPECT_NEAR(relative.at("models").at("resource").at("parts")[0].at("pose").at("position")[0].get<double>(), 6, 1e-9);

  const auto joint = compile(load("joint.sdf", "sdformat-model"), assets.string());
  EXPECT_EQ(joint.at("models").at("resource").at("parts").size(), 1u);
  EXPECT_EQ(joint.at("models").at("resource").at("unparsed")[0], "joint:j");
  EXPECT_THROW(compile(load("include.sdf", "sdformat-model"), assets.string()), SceneError);
}

TEST_F(Asset, NeighborScalesDoNotShareGeometry) {
  write("unit.dae", R"(<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <library_geometries><geometry id="g"><mesh>
    <source id="p"><float_array id="pa" count="12">1 0 0 1 0.1 0 1 0 0.1 1.1 0 0</float_array>
      <technique_common><accessor source="#pa" count="4" stride="3">
        <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
      </accessor></technique_common>
    </source>
    <vertices id="v"><input semantic="POSITION" source="#p"/></vertices>
    <triangles count="2"><input semantic="VERTEX" source="#v" offset="0"/><p>0 1 2 0 2 3</p></triangles>
  </mesh></geometry></library_geometries>
  <library_visual_scenes><visual_scene id="s"><node><instance_geometry url="#g"/></node></visual_scene></library_visual_scenes>
  <scene><instance_visual_scene url="#s"/></scene>
</COLLADA>)");
  const auto sdf = [](const char* scale) {
    return std::string("<sdf version=\"1.7\"><model name=\"m\"><link name=\"l\"><collision name=\"c\"><geometry><mesh><uri>unit.dae</uri><scale>") +
           scale + "</scale></mesh></geometry></collision></link></model></sdf>";
  };
  write("near-a.sdf", sdf("1.0000001 1 1"));
  write("near-b.sdf", sdf("1.0000002 1 1"));
  std::string obstacles = "[";
  for (int i = 0; i < 100; ++i) {
    if (i) obstacles += ",";
    obstacles += "{\"id\":\"a" + std::to_string(i) + "\",\"model\":\"near-a\"}";
  }
  obstacles += ",{\"id\":\"b\",\"model\":\"near-b\"}]";
  std::size_t reads = 0;
  const auto compiled = compile(scene("\"models\":{\"near-a\":{\"resource\":{\"format\":\"sdformat-model\",\"file\":\"near-a.sdf\"}},\"near-b\":{\"resource\":{\"format\":\"sdformat-model\",\"file\":\"near-b.sdf\"}}},\"obstacles\":" + obstacles), root.string(), &reads);
  EXPECT_EQ(reads, 3u);
  EXPECT_EQ(compiled.at("geometries").size(), 2u);
  EXPECT_EQ(compiled.at("obstacles").size(), 101u);
  const auto index_a = compiled.at("models").at("near-a").at("parts")[0].at("geometryIndex").get<std::size_t>();
  const auto index_b = compiled.at("models").at("near-b").at("parts")[0].at("geometryIndex").get<std::size_t>();
  EXPECT_NE(index_a, index_b);
  for (const auto& obstacle : compiled.at("obstacles")) {
    if (obstacle.at("model") != "near-a") continue;
    EXPECT_EQ(compiled.at("models").at("near-a").at("parts")[0].at("geometryIndex").get<std::size_t>(), index_a);
  }
  EXPECT_TRUE(nearVertex(compiled.at("geometries")[index_a].at("vertices"), 1.0000001, 0, 0));
  EXPECT_TRUE(nearVertex(compiled.at("geometries")[index_b].at("vertices"), 1.0000002, 0, 0));
}

TEST_F(Asset, AdjacentScalesInOneModelStayDistinct) {
  write("unit.dae", R"(<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <library_geometries><geometry id="g"><mesh>
    <source id="p"><float_array id="pa" count="12">1 0 0 1 0.1 0 1 0 0.1 1.1 0 0</float_array>
      <technique_common><accessor source="#pa" count="4" stride="3">
        <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
      </accessor></technique_common>
    </source>
    <vertices id="v"><input semantic="POSITION" source="#p"/></vertices>
    <triangles count="2"><input semantic="VERTEX" source="#v" offset="0"/><p>0 1 2 0 2 3</p></triangles>
  </mesh></geometry></library_geometries>
  <library_visual_scenes><visual_scene id="s"><node><instance_geometry url="#g"/></node></visual_scene></library_visual_scenes>
  <scene><instance_visual_scene url="#s"/></scene>
</COLLADA>)");
  write("scale-cache-collision.sdf", R"(<sdf version="1.7"><model name="m"><link name="l"><collision name="first"><geometry><mesh><uri>unit.dae</uri><scale>1 1 1</scale></mesh></geometry></collision><collision name="second"><geometry><mesh><uri>unit.dae</uri><scale>1.0000004 1 1</scale></mesh></geometry></collision></link></model></sdf>)");
  std::size_t reads = 0;
  const auto compiled = compile(scene("\"models\":{\"resource\":{\"resource\":{\"format\":\"sdformat-model\",\"file\":\"scale-cache-collision.sdf\"}}},\"obstacles\":[{\"id\":\"instance0\",\"model\":\"resource\"}]"), root.string(), &reads);
  const auto& parts = compiled.at("models").at("resource").at("parts");
  ASSERT_EQ(parts.size(), 2u);
  EXPECT_EQ(compiled.at("geometries").size(), 2u);
  EXPECT_EQ(reads, 2u);
  EXPECT_NE(parts[0].at("geometryIndex"), parts[1].at("geometryIndex"));
  EXPECT_NEAR(compiled.at("geometries")[parts[0].at("geometryIndex").get<std::size_t>()].at("vertices")[0][0].get<double>(), 1, 1e-12);
  EXPECT_NEAR(compiled.at("geometries")[parts[1].at("geometryIndex").get<std::size_t>()].at("vertices")[0][0].get<double>(), 1.0000004, 1e-12);
}

TEST_F(Asset, IncludesAreRejectedBeforeExpansion) {
  const auto included = root / "included";
  fs::create_directories(included);
  write("included/model.config", "<model><name>child</name><sdf version=\"1.7\">model.sdf</sdf></model>");
  write("included/model.sdf", R"(<sdf version="1.7"><model name="child"><link name="l"><collision name="c"><geometry><box><size>1 2 3</size></box></geometry></collision></link></model></sdf>)");
  write("valid-include-modeldir.sdf", "<sdf version=\"1.7\"><model name=\"m\"><link name=\"l\"><collision name=\"c\"><geometry><box><size>1 2 3</size></box></geometry></collision></link><include><uri>" + included.string() + "</uri><pose>5 0 0 0 0 0</pose></include></model></sdf>");
  EXPECT_THROW(compile(scene("\"models\":{\"resource\":{\"resource\":{\"format\":\"sdformat-model\",\"file\":\"valid-include-modeldir.sdf\"}}},\"obstacles\":[{\"id\":\"instance0\",\"model\":\"resource\"}]"), root.string()), SceneError);
  const auto outside = root.parent_path() / "outside-asset-root";
  std::string escaped;
  for (const char character : outside.string()) escaped += character == '/' ? "&#47;" : std::string(1, character);
  write("include-xml-entity-escape.sdf", "<sdf version=\"1.7\"><model name=\"m\"><link name=\"l\"><collision name=\"c\"><geometry><box><size>1 2 3</size></box></geometry></collision></link><include><uri>" + escaped + "</uri><pose>5 0 0 0 0 0</pose></include></model></sdf>");
  EXPECT_THROW(compile(scene("\"models\":{\"resource\":{\"resource\":{\"format\":\"sdformat-model\",\"file\":\"include-xml-entity-escape.sdf\"}}},\"obstacles\":[{\"id\":\"instance0\",\"model\":\"resource\"}]"), root.string()), SceneError);
}

TEST_F(Asset, RootModelPoseStaysOnTheInstance) {
  write("rooted.sdf", R"(<?xml version="1.0"?><sdf version="1.7"><model name="m"><pose>10 0 0 0 0 0</pose>
<link name="solid"><collision name="c"><geometry><box><size>1 1 1</size></box></geometry></collision></link>
<link name="paint"><visual name="v"><geometry><box><size>0.2 0.2 0.2</size></box></geometry></visual></link>
</model></sdf>)");
  const auto compiled = compile(scene("\"models\":{\"resource\":{\"resource\":{\"format\":\"sdformat-model\",\"file\":\"rooted.sdf\"}}},\"obstacles\":[{\"id\":\"placed\",\"model\":\"resource\",\"pose\":{\"position\":[4,0,0]}}]"), root.string());
  ASSERT_EQ(compiled.at("models").at("resource").at("parts").size(), 1u);
  EXPECT_DOUBLE_EQ(compiled.at("models").at("resource").at("parts")[0].at("pose").at("position")[0].get<double>(), 0);
  EXPECT_DOUBLE_EQ(compiled.at("obstacles")[0].at("pose").at("position")[0].get<double>(), 4);
  write("visual-only.sdf", R"(<?xml version="1.0"?><sdf version="1.7"><model name="m"><link name="paint"><visual name="v"><geometry><box><size>1 1 1</size></box></geometry></visual></link></model></sdf>)");
  EXPECT_THROW(compile(scene("\"models\":{\"resource\":{\"resource\":{\"format\":\"sdformat-model\",\"file\":\"visual-only.sdf\"}}},\"obstacles\":[]"), root.string()), SceneError);
}

TEST(Compiled, RejectsEscapingResourcePaths) {
  EXPECT_THROW(document(scene("\"models\":{\"m\":{\"resource\":{\"format\":\"mesh\",\"file\":\"../secret.dae\"}}},\"obstacles\":[]")), SceneError);
  EXPECT_THROW(compile(scene("\"ground\":{\"mode\":\"none\"},\"obstacles\":[]")).at("ground").at("z"), nlohmann::json::out_of_range);
}

}  // namespace
}  // namespace xgc_scene
