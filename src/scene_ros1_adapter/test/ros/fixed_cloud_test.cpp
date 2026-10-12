#include "xgc_scene/fixed_cloud.hpp"
#include "xgc_scene/yaml_io.hpp"

#include <gtest/gtest.h>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <unistd.h>

namespace xgc_scene {
namespace {

struct CloudAsset {
  std::filesystem::path directory;
  Json selected;
  CloudAsset() {
    char name[] = "/tmp/xgc-fixed-cloud-XXXXXX";
    if (!::mkdtemp(name)) throw std::runtime_error("Cannot create cloud fixture");
    directory = name;
    selected = {{"assetDirectory", directory.string()}, {"pointCloud", {
      {"file", "original.xyz.f32"}, {"format", "xyz-f32-le"}, {"frame", "world"}}}};
  }
  ~CloudAsset() { std::error_code error; std::filesystem::remove_all(directory, error); }
  void write(const std::string& data) {
    std::ofstream file(directory / "original.xyz.f32", std::ios::binary);
    file.write(data.data(), data.size());
    selected["pointCloud"]["sha256"] = digest(data);
  }
};

TEST(FixedCloud, PreservesSealedOriginalBytesAndStandardPointFields) {
  CloudAsset asset;
  const float points[] = {-1.5f, 2.0f, 3.0f, 4.0f, -5.0f, 6.0f};
  std::string data(sizeof(points), '\0');
  std::memcpy(data.data(), points, sizeof(points));
  asset.write(data);
  const auto message = fixedPointCloud(asset.selected);
  EXPECT_EQ(std::string(message.data.begin(), message.data.end()), data);
  EXPECT_EQ(message.header.frame_id, "world");
  EXPECT_TRUE(message.header.stamp.isZero());
  EXPECT_EQ(message.height, 1u);
  EXPECT_EQ(message.width, 2u);
  EXPECT_EQ(message.point_step, 12u);
  EXPECT_EQ(message.row_step, data.size());
  EXPECT_FALSE(message.is_bigendian);
  ASSERT_EQ(message.fields.size(), 3u);
  const char* names[] = {"x", "y", "z"};
  for (std::size_t i = 0; i < 3; ++i) {
    EXPECT_EQ(message.fields[i].name, names[i]);
    EXPECT_EQ(message.fields[i].offset, 4 * i);
    EXPECT_EQ(message.fields[i].datatype, sensor_msgs::PointField::FLOAT32);
    EXPECT_EQ(message.fields[i].count, 1u);
  }
}

TEST(FixedCloud, RejectsChangedBytesAndIncompletePointsInNativeOwner) {
  CloudAsset asset;
  asset.write(std::string(12, '\0'));
  asset.selected["pointCloud"]["sha256"] = digest("different content");
  EXPECT_THROW(fixedPointCloud(asset.selected), SceneError);
  asset.write(std::string(13, '\0'));
  EXPECT_THROW(fixedPointCloud(asset.selected), SceneError);
}

}  // namespace
}  // namespace xgc_scene
