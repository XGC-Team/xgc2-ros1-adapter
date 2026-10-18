#include <xgc2_math/geometry/geometry_json.hpp>
#include <sstream>
#include "xgc_scene/document.hpp"

#include "placement_json.hpp"
#include "scalars.hpp"

#include <xgc2_math/scene/convex_mesh.hpp>

#include <algorithm>
#include <cmath>
#include <regex>
#include <set>

namespace xgc_scene {

const Json& fields(const Json& value, const std::vector<std::string>& allowed,
                   const std::vector<std::string>& required) {
  if (!value.is_object()) throw SceneError("Expected an object");
  std::set<std::string> unknown, missing;
  for (auto it = value.begin(); it != value.end(); ++it)
    if (std::find(allowed.begin(), allowed.end(), it.key()) == allowed.end()) unknown.insert(it.key());
  for (const auto& key : required) if (!value.contains(key)) missing.insert(key);
  if (!unknown.empty() || !missing.empty())
    throw SceneError("Invalid fields: unknown=" + detail::pythonRepr(Json(unknown)) +
                     ", missing=" + detail::pythonRepr(Json(missing)));
  return value;
}

double number(const Json& value, const std::string& label, double minimum, double maximum) {
  if (!value.is_number()) throw SceneError(label + " must be finite");
  const double result = value.get<double>();
  if (!std::isfinite(result)) throw SceneError(label + " must be finite");
  if (!(minimum <= result && result <= maximum)) throw SceneError(label + " is outside the supported range");
  return result;
}

Json vector(const Json& value, const std::string& label, std::size_t count) {
  if (!value.is_array() || value.size() != count)
    throw SceneError(label + " must have " + std::to_string(count) + " components");
  Json result = Json::array();
  for (const auto& item : value) result.push_back(number(item, label));
  return result;
}

std::string identifier(const Json& value, const std::string& label) {
  static const std::regex pattern("[A-Za-z0-9][A-Za-z0-9_.-]{0,127}");
  if (!value.is_string() || !std::regex_match(value.get_ref<const std::string&>(), pattern))
    throw SceneError(label + " must be a stable nonempty identifier");
  return value.get<std::string>();
}

Json pose(const Json& value) {
  fields(value, {"position", "orientation"});
  Json position = vector(value.value("position", Json::array({0, 0, 0})), "position");
  Json rotation = vector(value.value("orientation", Json::array({0, 0, 0, 1})), "orientation", 4);
  double squared = 0;
  for (const auto& item : rotation) { const double x = item.get<double>(); squared += x * x; }
  const double norm = std::sqrt(squared);
  if (norm < 1e-12) throw SceneError("Orientation quaternion must be nonzero");
  for (auto& item : rotation) item = item.get<double>() / norm;
  return Json{{"position", position}, {"orientation", rotation}};
}

Json convex(const Json& value) {
  const auto raw_vertices = value.value("vertices", Json(nullptr));
  const auto raw_indices = value.value("triangles", Json(nullptr));
  if (!raw_vertices.is_array() || raw_vertices.size() < 4 || raw_vertices.size() > 4096)
    throw SceneError("Convex geometry requires 4 to 4096 vertices");
  Json vertices = Json::array();
  std::vector<Eigen::Vector3d> positions;
  for (const auto& point : raw_vertices) {
    vertices.push_back(vector(point, "vertex"));
    positions.push_back(detail::toVector(vertices.back()));
  }
  std::vector<std::int64_t> indices;
  if (raw_indices.is_array()) {
    for (const auto& index : raw_indices) {
      // Invalid JSON index types go to Math as an invalid index. Math then
      // keeps Python's check order: duplicate vertices, triplets, indices.
      if (!index.is_number_integer() || index.get<double>() < 0 || index.get<double>() >= positions.size()) indices.push_back(-1);
      else indices.push_back(index.get<std::int64_t>());
    }
  }
  try {
    const auto triangles = xgc2_math::scene::validateConvexMesh(positions, indices);
    return Json{{"type", "convex"}, {"vertices", vertices}, {"triangles", triangles}};
  } catch (const std::invalid_argument& error) { throw SceneError(error.what()); }
}

Json geometry(const Json& value) {
  if (!value.is_object()) throw SceneError("Geometry must be an object");
  const Json kind = value.value("type", Json(nullptr));
  if (kind == "box") {
    fields(value, {"type", "size"}, {"size"});
    const Json size = vector(value.at("size"), "box size");
    for (const auto& side : size) if (side.get<double>() <= 0) throw SceneError("Box side lengths must be positive");
    return Json{{"type", kind}, {"size", size}};
  }
  if (kind == "sphere" || kind == "cylinder" || kind == "capsule") {
    const std::vector<std::string> allowed = kind == "sphere" ? std::vector<std::string>{"type", "radius"} :
                                                              std::vector<std::string>{"type", "radius", "height"};
    fields(value, allowed, allowed);
    const double radius = number(value.at("radius"), "radius", 1e-6, 1e4);
    Json result{{"type", kind}, {"radius", radius}};
    if (kind != "sphere") result["height"] = number(value.at("height"), "height", kind == "capsule" ? 0 : 1e-6, 1e4);
    return result;
  }
  if (kind == "ring") {
    fields(value, {"type", "radii", "tubeRadius"}, {"radii", "tubeRadius"});
    Json radii = vector(value.at("radii"), "ring radii", 2);
    for (auto& radius : radii) radius = number(radius, "ring radii", 1e-6, 1e4);
    const double tube = number(value.at("tubeRadius"), "ring tube radius", 1e-6, 1e4);
    if (tube >= std::min(radii[0].get<double>(), radii[1].get<double>()))
      throw SceneError("Ring tube radius must be smaller than both radii");
    return Json{{"type", kind}, {"radii", radii}, {"tubeRadius", tube}};
  }
  if (kind == "convex" && value.contains("triangles") && (!value.at("triangles").is_array() || !value.at("triangles").empty())) {
    fields(value, {"type", "vertices", "triangles"}, {"vertices", "triangles"});
    return convex(value);
  }
  if (kind.is_string() && std::any_of(std::begin(xgc2_math::geometry::geometryRegistry), std::end(xgc2_math::geometry::geometryRegistry),
      [&](const auto& entry) { return entry.name == kind.get<std::string>(); })) {
    try {
      const auto type = xgc2_math::geometry::geometryTypeFromName(kind.get<std::string>());
      const auto spec = xgc2_math::geometry::geometryTypeInfo(static_cast<std::uint32_t>(type));
      std::vector<std::string> keys{"type"};
      std::string declarations(spec.fields);
      std::istringstream input(declarations);
      std::string key;
      while (std::getline(input, key, ',')) keys.push_back(key.substr(0, key.find('[')));
      fields(value, keys, {"type"});
      return xgc2_math::geometry::geometryToJson<Json>(xgc2_math::geometry::geometryFromJson(value));
    } catch (const std::exception& error) { throw SceneError(error.what()); }
  }
  throw SceneError("Unsupported geometry " + detail::pythonRepr(kind) + "; no shape substitution is performed");
}

Json motion(const Json& value) {
  if (!value.is_object()) throw SceneError("Motion must be an object");
  const Json kind = value.value("type", Json("hold"));
  if (kind == "hold") { fields(value, {"type"}); return Json{{"type", kind}}; }
  if (kind == "constant_twist") {
    fields(value, {"type", "linear", "angular"});
    const auto linear = vector(value.value("linear", Json::array({0, 0, 0})), "linear velocity");
    const auto angular = vector(value.value("angular", Json::array({0, 0, 0})), "angular velocity");
    return Json{{"type", kind}, {"linear", linear}, {"angular", angular}};
  }
  if (kind == "ping_pong") {
    fields(value, {"type", "point_a", "point_b", "speed"}, {"point_a", "point_b", "speed"});
    const Json a = vector(value.at("point_a"), "point A"), b = vector(value.at("point_b"), "point B");
    double squared = 0;
    for (std::size_t i = 0; i < 3; ++i) squared += std::pow(a[i].get<double>() - b[i].get<double>(), 2);
    if (squared < 1e-12) throw SceneError("Ping-pong endpoints must differ");
    const double speed = number(value.at("speed"), "speed", 1e-6, 1000);
    return Json{{"type", kind}, {"point_a", a}, {"point_b", b}, {"speed", speed}};
  }
  if (kind == "circle") {
    fields(value, {"type", "center", "radius", "angular_speed", "phase"}, {"center", "radius", "angular_speed"});
    const auto center = vector(value.at("center"), "circle center");
    const double radius = number(value.at("radius"), "circle radius", 1e-6, 1e4);
    const double angular_speed = number(value.at("angular_speed"), "angular speed", -1000, 1000);
    const double phase = number(value.value("phase", Json(0)), "phase");
    return Json{{"type", kind}, {"center", center}, {"radius", radius}, {"angular_speed", angular_speed}, {"phase", phase}};
  }
  if (kind == "waypoints") {
    fields(value, {"type", "velocity", "orientation", "angular_velocity", "loop", "sin", "waypoints"}, {"waypoints"});
    const auto& points = value.at("waypoints");
    if (!points.is_array() || points.size() < 2 || points.size() > 4096)
      throw SceneError("Waypoints require 2 to 4096 positions");
    const auto flag = [&](const char* key, bool fallback) {
      if (!value.contains(key)) return fallback;
      if (!value.at(key).is_boolean()) throw SceneError(std::string(key) + " must be a boolean");
      return value.at(key).get<bool>();
    };
    Json result{{"type", kind}, {"velocity", number(value.value("velocity", Json(1.0)), "velocity", 1e-6, 1000)}};
    result["orientation"] = flag("orientation", true);
    result["angular_velocity"] = number(value.value("angular_velocity", Json(0.8)), "angular velocity", 1e-6, 1000);
    result["loop"] = flag("loop", false);
    result["sin"] = flag("sin", false);
    result["waypoints"] = Json::array();
    for (const auto& point : points) result["waypoints"].push_back(vector(point, "waypoint"));
    return result;
  }
  throw SceneError("Unsupported motion " + detail::pythonRepr(kind));
}

Json normalizeParts(const Json& parts) {
  if (!parts.is_array() || parts.empty() || parts.size() > 128) throw SceneError("Obstacle requires 1 to 128 parts");
  Json normalized = Json::array();
  std::set<std::string> ids;
  for (const auto& part : parts) {
    fields(part, {"id", "pose", "geometry", "color"}, {"id", "geometry"});
    const auto pid = identifier(part.at("id"), "Part ID");
    if (!ids.insert(pid).second) throw SceneError("Duplicate part ID " + pid);
    if (part.contains("color")) {
      const auto color = vector(part.at("color"), "color", 4);
      for (const auto& channel : color)
        if (channel.get<double>() < 0 || channel.get<double>() > 1) throw SceneError("Color channels must be between 0 and 1");
    }
    normalized.push_back(Json{{"id", pid}, {"pose", pose(part.value("pose", Json::object()))},
                              {"geometry", geometry(part.at("geometry"))},
                              {"color", Json::array({1.0, 0.5, 0.1, 1.0})}});
  }
  return normalized;
}

Json resourceRef(const Json& value) {
  fields(value, {"format", "file"}, {"format", "file"});
  const auto& format = value.at("format");
  if (format != "sdformat-model" && format != "gltf" && format != "mesh")
    throw SceneError("Model resource format must be sdformat-model, gltf, or mesh");
  if (!value.at("file").is_string()) throw SceneError("Model resource file must be asset-relative");
  const auto& file = value.at("file").get_ref<const std::string&>();
  const bool escaped = file.empty() || file.front() == '/' || file.find('\\') != std::string::npos ||
                       file.find('\0') != std::string::npos;
  if (!escaped) {
    std::size_t begin = 0;
    while (begin <= file.size()) {
      const auto end = file.find('/', begin);
      const auto segment = file.substr(begin, end == std::string::npos ? std::string::npos : end - begin);
      if (segment.empty() || segment == "." || segment == "..") {
        throw SceneError("Model resource file must be asset-relative");
      }
      if (end == std::string::npos) break;
      begin = end + 1;
    }
  } else throw SceneError("Model resource file must be asset-relative");
  return Json{{"format", format}, {"file", file}};
}

Json obstacle(const Json& value) {
  fields(value, {"id", "name", "pose", "parts", "motion", "model"}, {"id"});
  const auto oid = identifier(value.at("id"), "Obstacle ID");
  if (oid == "ground_plane") throw SceneError("ground_plane is owned by ground, not an obstacle");
  const Json name = value.value("name", Json(oid));
  if (!name.is_string()) throw SceneError("Obstacle name must have 1 to 256 characters");
  const auto characters = detail::utf8Characters(name.get_ref<const std::string&>());
  if (characters.size() > 256 || std::all_of(characters.begin(), characters.end(), [](const auto& c) {
        return detail::pythonWhitespace(detail::codepoint(c));
      })) throw SceneError("Obstacle name must have 1 to 256 characters");
  const bool has_parts = value.contains("parts");
  const bool has_model = value.contains("model");
  if (has_parts == has_model) throw SceneError("Obstacle requires parts or one model");
  const Json initial = pose(value.value("pose", Json::object()));
  const Json movement = motion(value.value("motion", Json{{"type", "hold"}}));
  if (movement.at("type") != "waypoints") {
    const auto start = xgc2_math::scene::motionStart(detail::toVector(initial.at("position")), detail::toMotion(movement));
    for (int i = 0; i < 3; ++i)
      if (std::abs(start[i] - initial.at("position")[i].get<double>()) > 1e-6)
        throw SceneError("Initial obstacle position must match its motion path start");
  }
  Json result{{"id", oid}, {"name", name}, {"pose", initial}};
  if (has_model) {
    result["model"] = identifier(value.at("model"), "Model ID");
  } else {
    result["parts"] = normalizeParts(value.at("parts"));
  }
  result["motion"] = movement;
  return result;
}

Json models(const Json& value) {
  if (!value.is_object()) throw SceneError("Models must be an object");
  Json result = Json::object();
  for (auto it = value.begin(); it != value.end(); ++it) {
    const auto name = identifier(Json(it.key()), "Model ID");
    fields(it.value(), {"parts", "resource"});
    const bool has_parts = it.value().contains("parts");
    const bool has_resource = it.value().contains("resource");
    if (has_parts == has_resource) throw SceneError("Model requires parts or one resource");
    if (has_resource) result[name] = Json{{"resource", resourceRef(it.value().at("resource"))}};
    else result[name] = Json{{"parts", normalizeParts(it.value().at("parts"))}};
  }
  return result;
}

Json ground(const Json& value) {
  if (!value.is_object() || !value.contains("mode")) throw SceneError("Ground requires a mode");
  const auto& mode = value.at("mode");
  if (mode == "default_plane") {
    fields(value, {"mode", "z"}, {"mode"});
    return Json{{"mode", mode}, {"z", number(value.value("z", Json(0.0)), "ground z")}};
  }
  if (mode == "none") {
    fields(value, {"mode"}, {"mode"});
    return Json{{"mode", mode}};
  }
  if (mode == "model") {
    fields(value, {"mode", "model", "pose"}, {"mode", "model", "pose"});
    return Json{{"mode", mode}, {"model", identifier(value.at("model"), "Model ID")}, {"pose", pose(value.at("pose"))}};
  }
  throw SceneError("Ground mode must be default_plane, model, or none");
}

const Json& checkObstacleSet(const Json& obstacles) {
  if (obstacles.size() > 512) throw SceneError("Scene supports at most 512 obstacles");
  std::set<std::string> ids;
  std::size_t parts = 0;
  for (const auto& item : obstacles) {
    ids.insert(item.at("id").get<std::string>());
    if (item.contains("parts")) parts += item.at("parts").size();
  }
  if (ids.size() != obstacles.size()) throw SceneError("Duplicate obstacle ID");
  if (parts > 4096) throw SceneError("Scene supports at most 4096 convex parts");
  return obstacles;
}

Json document(const Json& value) {
  fields(value, {"schema", "id", "frame", "obstacles", "models", "ground"}, {"schema", "id", "frame", "obstacles"});
  if (value.at("schema") != kSchema) throw SceneError("Unsupported scene schema");
  identifier(value.at("id"), "Scene ID");
  const auto& frame = value.at("frame");
  static const std::regex frame_pattern("[A-Za-z_][A-Za-z0-9_]*(/[A-Za-z_][A-Za-z0-9_]*)*");
  if (!frame.is_string() || frame.get_ref<const std::string&>().size() > 160 ||
      !std::regex_match(frame.get_ref<const std::string&>(), frame_pattern))
    throw SceneError("Scene frame must be explicit and have no leading slash");
  const auto& items = value.at("obstacles");
  if (!items.is_array() || items.size() > 512) throw SceneError("Scene supports at most 512 obstacles");
  Json obstacles = Json::array();
  for (const auto& item : items) obstacles.push_back(obstacle(item));
  checkObstacleSet(obstacles);
  Json defined = value.contains("models") ? models(value.at("models")) : Json::object();
  for (const auto& item : obstacles) {
    if (!item.contains("model")) continue;
    if (!defined.contains(item.at("model").get<std::string>()))
      throw SceneError("Obstacle model is not defined");
  }
  Json result{{"schema", kSchema}, {"id", value.at("id")}, {"frame", frame}, {"obstacles", obstacles}};
  if (value.contains("models")) result["models"] = std::move(defined);
  if (value.contains("ground")) {
    auto owned = ground(value.at("ground"));
    if (owned.at("mode") == "model" && !result.value("models", Json::object()).contains(owned.at("model").get<std::string>()))
      throw SceneError("Ground model is not defined");
    result["ground"] = std::move(owned);
  }
  return result;
}

}  // namespace xgc_scene
