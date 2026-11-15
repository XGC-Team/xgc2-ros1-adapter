#include "xgc_scene/fixed_cloud.hpp"
#include "xgc_scene/prepare.hpp"
#include "xgc_scene/yaml_io.hpp"

#include <cmath>
#include <cstring>

namespace xgc_scene {

sensor_msgs::PointCloud2 fixedPointCloud(const Json& selected) {
  const auto& source = selected.at("pointCloud");
  if (source.at("format") != "xyz-f32-le" || source.at("frame") != "world")
    throw SceneError("Fixed point cloud requires xyz-f32-le in the world frame");
  const std::filesystem::path directory = selected.at("assetDirectory").get<std::string>();
  const auto path = checkedPath((directory / source.at("file").get<std::string>()).string(), directory);
  const auto data = read(path, 48 * 1024 * 1024);
  if (data.size() % 12 != 0) throw SceneError("Fixed point cloud contains incomplete XYZ points");
  if (digest(data) != source.at("sha256"))
    throw SceneError("Fixed point cloud changed after it was sealed");
  sensor_msgs::PointCloud2 message;
  message.header.frame_id = "world";
  message.height = 1;
  message.width = data.size() / 12;
  message.is_bigendian = false;
  message.is_dense = true;
  message.point_step = 12;
  message.row_step = data.size();
  const char* names[] = {"x", "y", "z"};
  for (std::size_t i = 0; i < 3; ++i) {
    sensor_msgs::PointField field;
    field.name = names[i];
    field.offset = i * 4;
    field.datatype = sensor_msgs::PointField::FLOAT32;
    field.count = 1;
    message.fields.push_back(field);
  }
  message.data.assign(data.begin(), data.end());
  return message;
}

std::optional<sensor_msgs::PointCloud2> frozenFixedCloud(const Json& session,
                                                         const std::filesystem::path& asset_directory) {
  const auto asset = session.find("sceneAsset");
  if (asset == session.end() || !asset->is_object() || !asset->contains("pointCloud")) return std::nullopt;
  auto message = fixedPointCloud(Json{{"assetDirectory", asset_directory.string()}, {"pointCloud", asset->at("pointCloud")}});
  const auto asset_offset = asset->find("localizationOffset");
  const auto target_offset = session.find("localizationOffset");
  if (asset_offset != asset->end() && target_offset != session.end() &&
      !asset_offset->is_null() && !target_offset->is_null()) {
    if (!asset_offset->is_object() || !target_offset->is_object())
      throw SceneError("localizationOffset must be an object");
    auto read_coord = [](const Json& obj, const char* key) -> double {
      if (!obj.contains(key) || !obj.at(key).is_number())
        throw SceneError("localizationOffset requires finite x, y, z");
      const double v = obj.at(key).get<double>();
      if (!std::isfinite(v)) throw SceneError("localizationOffset requires finite x, y, z");
      return v;
    };
    const float dx = static_cast<float>(read_coord(*target_offset, "x") - read_coord(*asset_offset, "x"));
    const float dy = static_cast<float>(read_coord(*target_offset, "y") - read_coord(*asset_offset, "y"));
    const float dz = static_cast<float>(read_coord(*target_offset, "z") - read_coord(*asset_offset, "z"));
    if (dx != 0.0f || dy != 0.0f || dz != 0.0f) {
      for (std::size_t off = 0; off + 12 <= message.data.size(); off += 12) {
        float xyz[3];
        std::memcpy(xyz, message.data.data() + off, 12);
        xyz[0] += dx;
        xyz[1] += dy;
        xyz[2] += dz;
        std::memcpy(message.data.data() + off, xyz, 12);
      }
    }
  }
  return message;
}

}  // namespace xgc_scene
