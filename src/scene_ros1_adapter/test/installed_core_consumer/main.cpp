#include <xgc_scene/document.hpp>
#include <xgc_scene/motion.hpp>
#include <xgc_scene/yaml_io.hpp>

#include <fstream>

int main() {
  using namespace xgc_scene;
  const auto path = std::filesystem::current_path() / "source.yaml";
  std::ofstream(path) << "schema: xgc2.scene.v1\nid: installed\nframe: world\nobstacles: []\n";
  const auto source = load(path).first;
  const auto compiled = compile(source, path.parent_path().string());
  const auto moving = obstacle(Json{{"id", "moving"}, {"parts", Json::array({
      Json{{"id", "body"}, {"geometry", {{"type", "box"}, {"size", Json::array({1, 2, 3})}}}}})},
      {"motion", {{"type", "constant_twist"}, {"linear", Json::array({1, 0, 0})}}}});
  const auto current = state(moving, 2.0, true);
  return compiled.at("schema") == kCompiledSchema && current.at("pose").at("position").at(0) == 2.0 ? 0 : 1;
}
