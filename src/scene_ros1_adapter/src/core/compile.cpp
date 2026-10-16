#include "xgc_scene/document.hpp"

#include "placement_json.hpp"
#include "xgc_scene/json_io.hpp"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#include <sdf/Box.hh>
#include <sdf/Collision.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Error.hh>
#include <sdf/Geometry.hh>
#include <sdf/Link.hh>
#include <sdf/Mesh.hh>
#include <sdf/Model.hh>
#include <sdf/Root.hh>
#include <sdf/Sphere.hh>
#include <sdf/parser.hh>

#include <pugixml.hpp>

#include <filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <tuple>
#include <utility>

namespace xgc_scene {
namespace {

namespace fs = std::filesystem;

const Json kAmber = Json::array({1.0, 0.5, 0.1, 1.0});

struct Compiler {
  Json geometries = Json::array();
  std::map<std::string, std::size_t> indexes;
  // Parsed mesh files live for this compile only. Callers borrow them.
  std::map<std::string, std::vector<Json>> files;
  // K is one source geometry definition: file path, mesh piece, and exact scale.
  // The same numeric mesh at another path stays a separate definition. Do not hash or dump mesh bytes to intern them.
  using MeshKey = std::tuple<std::string, std::size_t, double, double, double>;
  std::map<MeshKey, std::size_t> mesh_geometry;
  std::size_t reads = 0;

  std::size_t intern(const Json& geometry) {
    const auto key = dumpJson(geometry);
    const auto found = indexes.find(key);
    if (found != indexes.end()) return found->second;
    const auto index = geometries.size();
    geometries.push_back(geometry);
    indexes.emplace(key, index);
    return index;
  }

  static MeshKey meshKey(const fs::path& path, std::size_t piece, double sx, double sy, double sz) {
    return MeshKey{path.generic_string(), piece, sx, sy, sz};
  }

  std::optional<std::size_t> cachedMesh(const fs::path& path, std::size_t piece, double sx, double sy, double sz) const {
    const auto found = mesh_geometry.find(meshKey(path, piece, sx, sy, sz));
    if (found == mesh_geometry.end()) return {};
    return found->second;
  }

  std::size_t storeMesh(const fs::path& path, std::size_t piece, double sx, double sy, double sz, Json geometry) {
    const auto index = geometries.size();
    geometries.push_back(std::move(geometry));
    mesh_geometry.emplace(meshKey(path, piece, sx, sy, sz), index);
    return index;
  }
};

xgc2_math::Pose3 poseFromSdf(const gz::math::Pose3d& pose) {
  xgc2_math::Pose3 result;
  result.position = {pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z()};
  result.orientation = Eigen::Quaterniond(pose.Rot().W(), pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z());
  return result;
}

void rejectSdf(const sdf::Errors& errors) {
  if (errors.empty()) return;
  throw SceneError("SDF pose is unsupported: " + errors.front().Message());
}

fs::path insideRoot(const fs::path& root, const fs::path& base, const std::string& relative) {
  fs::path file(relative);
  if (relative.rfind("model://", 0) == 0 || file.is_absolute())
    throw SceneError("Scene resource URI must be asset-relative");
  const auto full = (base / file).lexically_normal();
  const auto stayed = full.lexically_relative(root.lexically_normal());
  if (stayed.empty() || stayed.begin()->native() == "..") throw SceneError("Scene resource URI must stay inside the asset directory");
  if (!fs::is_regular_file(full)) throw SceneError("Scene resource file is missing");
  return full;
}

std::string readBytes(const fs::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) throw SceneError("Scene resource file is missing");
  return std::string((std::istreambuf_iterator<char>(input)), {});
}

void rejectIncludes(const std::string& bytes) {
  pugi::xml_document document;
  const auto parsed = document.load_buffer(bytes.data(), bytes.size(), pugi::parse_default);
  if (!parsed) throw SceneError(std::string("SDF resource could not be parsed: ") + parsed.description());
  const auto include = document.find_node([](const pugi::xml_node& node) {
    return std::string(node.name()) == "include";
  });
  if (include) throw SceneError("SDF include is unsupported");
}

sdf::Root loadSdfBytes(const std::string& bytes) {
  sdf::SDFPtr parsed(new sdf::SDF());
  if (!sdf::init(parsed)) throw SceneError("SDF resource could not be parsed");
  sdf::Errors errors;
  if (!sdf::readString(bytes, parsed, errors)) {
    const auto message = errors.empty() ? std::string("SDF resource could not be parsed") : errors.front().Message();
    throw SceneError("SDF resource could not be parsed: " + message);
  }
  sdf::Root sdf_root;
  const auto loaded = sdf_root.Load(parsed);
  if (!loaded.empty()) throw SceneError("SDF resource could not be parsed: " + loaded.front().Message());
  return sdf_root;
}

const std::vector<Json>& readMeshFile(Compiler& compiler, const fs::path& path) {
  const auto key = path.string();
  const auto cached = compiler.files.find(key);
  if (cached != compiler.files.end()) return cached->second;
  ++compiler.reads;
  Assimp::Importer importer;
  const auto* scene = importer.ReadFile(key, aiProcess_Triangulate | aiProcess_SortByPType);
  if (!scene || !scene->mRootNode) throw SceneError("Mesh resource could not be parsed");
  std::vector<Json> pieces;
  const auto visit = [&](auto&& self, const aiNode* node, const aiMatrix4x4& parent) -> void {
    const aiMatrix4x4 global = parent * node->mTransformation;
    for (unsigned index = 0; index < node->mNumMeshes; ++index) {
      const aiMesh* mesh = scene->mMeshes[node->mMeshes[index]];
      Json vertices = Json::array();
      for (unsigned vertex = 0; vertex < mesh->mNumVertices; ++vertex) {
        const aiVector3D point = global * mesh->mVertices[vertex];
        vertices.push_back(Json::array({point.x, point.y, point.z}));
      }
      Json triangles = Json::array();
      for (unsigned face = 0; face < mesh->mNumFaces; ++face) {
        if (mesh->mFaces[face].mNumIndices != 3) throw SceneError("Mesh face must be a triangle");
        for (unsigned corner = 0; corner < 3; ++corner) triangles.push_back(mesh->mFaces[face].mIndices[corner]);
      }
      pieces.push_back(geometry(Json{{"type", "mesh"}, {"vertices", std::move(vertices)}, {"triangles", std::move(triangles)}}));
    }
    for (unsigned child = 0; child < node->mNumChildren; ++child) self(self, node->mChildren[child], global);
  };
  visit(visit, scene->mRootNode, aiMatrix4x4());
  if (pieces.empty()) throw SceneError("Mesh resource has no triangles");
  return compiler.files.emplace(key, std::move(pieces)).first->second;
}

Json scaleBakedMesh(const Json& source, double sx, double sy, double sz) {
  if (sx == 1 && sy == 1 && sz == 1) return source;
  if (!(sx > 0 && sy > 0 && sz > 0)) throw SceneError("Mesh scale must be positive");
  Json vertices = Json::array();
  for (const auto& vertex : source.at("vertices")) {
    vertices.push_back(Json::array({vertex.at(0).get<double>() * sx, vertex.at(1).get<double>() * sy,
                                    vertex.at(2).get<double>() * sz}));
  }
  return geometry(Json{{"type", "mesh"}, {"vertices", std::move(vertices)}, {"triangles", source.at("triangles")}});
}

Json compiledPart(Compiler& compiler, const std::string& id, const xgc2_math::Pose3& pose, const Json& geometry_value) {
  return Json{{"id", id}, {"pose", detail::fromPose(pose)}, {"geometryIndex", compiler.intern(geometry_value)}, {"color", kAmber}};
}

void noteUnparsed(const sdf::ElementPtr& element, Json& notes) {
  if (!element) return;
  for (auto child = element->GetFirstElement(); child; child = child->GetNextElement()) {
    const auto& tag = child->GetName();
    // A resolved <include> is already expanded by sdformat and is not left on this element.
    if (tag != "plugin" && tag != "joint" && tag != "actor" && tag != "light") continue;
    std::string label = tag;
    if (child->GetAttributeSet("name")) label += ":" + child->GetAttribute("name")->GetAsString();
    notes.push_back(label);
  }
}

Json collisionGeometry(Compiler& compiler, const sdf::Geometry& body, const fs::path& root, const fs::path& sdf_dir,
                       const std::string& id, const xgc2_math::Pose3& pose) {
  switch (body.Type()) {
    case sdf::GeometryType::BOX: {
      const auto size = body.BoxShape()->Size();
      return compiledPart(compiler, id, pose, geometry(Json{{"type", "box"}, {"size", Json::array({size.X(), size.Y(), size.Z()})}}));
    }
    case sdf::GeometryType::SPHERE:
      return compiledPart(compiler, id, pose, geometry(Json{{"type", "sphere"}, {"radius", body.SphereShape()->Radius()}}));
    case sdf::GeometryType::CYLINDER:
      return compiledPart(compiler, id, pose,
                          geometry(Json{{"type", "cylinder"}, {"radius", body.CylinderShape()->Radius()},
                                        {"height", body.CylinderShape()->Length()}}));
    case sdf::GeometryType::MESH: {
      const auto* mesh = body.MeshShape();
      const auto scale = mesh->Scale();
      const auto file = insideRoot(root, sdf_dir, mesh->Uri());
      const auto& pieces = readMeshFile(compiler, file);
      Json parts = Json::array();
      for (std::size_t index = 0; index < pieces.size(); ++index) {
        const auto piece_id = index == 0 ? id : id + "_" + std::to_string(index);
        const auto sx = scale.X(), sy = scale.Y(), sz = scale.Z();
        auto geometry_index = compiler.cachedMesh(file, index, sx, sy, sz);
        if (!geometry_index) {
          Json derived = (sx == 1 && sy == 1 && sz == 1) ? pieces[index] : scaleBakedMesh(pieces[index], sx, sy, sz);
          geometry_index = compiler.storeMesh(file, index, sx, sy, sz, std::move(derived));
        }
        parts.push_back(Json{{"id", piece_id}, {"pose", detail::fromPose(pose)}, {"geometryIndex", *geometry_index}, {"color", kAmber}});
      }
      return parts;
    }
    default:
      throw SceneError("SDF collision geometry is unsupported");
  }
}

xgc2_math::Pose3 modelPose(const sdf::SemanticPose& semantic) {
  gz::math::Pose3d resolved;
  // "__model__" is this model's frame. It includes nested/link/frame/relative_to
  // and does not add the root model's own pose; instance placement owns that.
  rejectSdf(semantic.Resolve(resolved, "__model__"));
  return poseFromSdf(resolved);
}

void collectModel(Compiler& compiler, const sdf::Model& model, const xgc2_math::Pose3& parent, bool is_root,
                  const fs::path& root, const fs::path& sdf_dir, const std::string& prefix, Json& parts, Json& notes) {
  noteUnparsed(model.Element(), notes);
  for (std::uint64_t link_index = 0; link_index < model.LinkCount(); ++link_index) {
    const auto* link = model.LinkByIndex(link_index);
    for (std::uint64_t collision_index = 0; collision_index < link->CollisionCount(); ++collision_index) {
      const auto* collision = link->CollisionByIndex(collision_index);
      if (!collision->Geom()) throw SceneError("SDF collision geometry is unsupported");
      const auto local = modelPose(collision->SemanticPose());
      const auto pose = is_root ? local : xgc2_math::compose(parent, local);
      const auto id = prefix + link->Name() + "_" + collision->Name();
      const auto part = collisionGeometry(compiler, *collision->Geom(), root, sdf_dir, id, pose);
      if (part.is_array()) {
        for (const auto& item : part) parts.push_back(item);
      } else parts.push_back(part);
    }
  }
  for (std::uint64_t nested = 0; nested < model.ModelCount(); ++nested) {
    const auto* child = model.ModelByIndex(nested);
    const auto nested_local = modelPose(child->SemanticPose());
    const auto nested_pose = is_root ? nested_local : xgc2_math::compose(parent, nested_local);
    collectModel(compiler, *child, nested_pose, false, root, sdf_dir, prefix + child->Name() + "_", parts, notes);
  }
}

Json bakeResource(Compiler& compiler, const fs::path& root, const Json& resource) {
  const auto path = insideRoot(root, root, resource.at("file").get<std::string>());
  const auto format = resource.at("format").get<std::string>();
  Json parts = Json::array();
  Json notes = Json::array();
  if (format == "mesh" || format == "gltf") {
    const auto& pieces = readMeshFile(compiler, path);
    for (std::size_t index = 0; index < pieces.size(); ++index) {
      auto geometry_index = compiler.cachedMesh(path, index, 1, 1, 1);
      if (!geometry_index) geometry_index = compiler.storeMesh(path, index, 1, 1, 1, pieces[index]);
      parts.push_back(Json{{"id", "mesh_" + std::to_string(index)}, {"pose", detail::fromPose({})},
                           {"geometryIndex", *geometry_index}, {"color", kAmber}});
    }
  } else if (format == "sdformat-model") {
    ++compiler.reads;
    const auto bytes = readBytes(path);
    rejectIncludes(bytes);
    sdf::Root sdf_root = loadSdfBytes(bytes);
    if (sdf_root.WorldCount() != 0) throw SceneError("SDF world is unsupported");
    if (sdf_root.ModelCount() != 1) throw SceneError("SDF resource requires one model");
    const auto* model = sdf_root.ModelByIndex(0);
    collectModel(compiler, *model, {}, true, root, path.parent_path(), "", parts, notes);
  } else throw SceneError("Model resource format must be sdformat-model, gltf, or mesh");
  if (!parts.is_array() || parts.empty()) throw SceneError("Model resource has no collision geometry");
  Json result{{"parts", std::move(parts)}, {"resource", Json{{"format", format}, {"file", path.string()}}}};
  if (!notes.empty()) result["unparsed"] = std::move(notes);
  return result;
}

Json partsFromAnalytic(Compiler& compiler, const Json& parts) {
  Json result = Json::array();
  for (const auto& part : parts) {
    result.push_back(Json{{"id", part.at("id")}, {"pose", part.at("pose")},
                          {"geometryIndex", compiler.intern(part.at("geometry"))}, {"color", part.at("color")}});
  }
  return result;
}

}  // namespace

Json compile(const Json& value, const std::string& resource_root, std::size_t* resource_reads) {
  const Json source = document(value);
  const bool uses_resource = [&] {
    for (const auto& model : source.value("models", Json::object()))
      if (model.contains("resource")) return true;
    return false;
  }();
  fs::path root(resource_root);
  if (uses_resource && (resource_root.empty() || !root.is_absolute() || !fs::is_directory(root)))
    throw SceneError("Compiled resources require an absolute asset directory");
  Compiler compiler;
  Json compiled_models = Json::object();
  const Json defined = source.value("models", Json::object());
  for (auto it = defined.begin(); it != defined.end(); ++it) {
    if (it.value().contains("resource")) compiled_models[it.key()] = bakeResource(compiler, root, it.value().at("resource"));
    else compiled_models[it.key()] = Json{{"parts", partsFromAnalytic(compiler, it.value().at("parts"))}};
  }
  Json obstacles = Json::array();
  for (const auto& item : source.at("obstacles")) {
    std::string model = item.value("model", Json("")).is_string() ? item.value("model", Json("")).get<std::string>() : "";
    if (!item.contains("model")) {
      model = "__inline_" + item.at("id").get<std::string>();
      if (compiled_models.contains(model)) throw SceneError("Inline model name collides with an authored model");
      compiled_models[model] = Json{{"parts", partsFromAnalytic(compiler, item.at("parts"))}};
    }
    obstacles.push_back(Json{{"id", item.at("id")}, {"name", item.at("name")}, {"model", model},
                             {"pose", item.at("pose")}, {"motion", item.at("motion")}});
  }
  Json result{{"schema", kCompiledSchema}, {"id", source.at("id")}, {"frame", source.at("frame")},
              {"ground", source.value("ground", Json{{"mode", "default_plane"}, {"z", 0.0}})},
              {"geometries", std::move(compiler.geometries)}, {"models", std::move(compiled_models)},
              {"obstacles", std::move(obstacles)}};
  if (resource_reads) *resource_reads = compiler.reads;
  return result;
}

}  // namespace xgc_scene
