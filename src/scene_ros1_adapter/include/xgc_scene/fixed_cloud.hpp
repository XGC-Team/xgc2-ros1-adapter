#pragma once

#include "xgc_scene/common.hpp"
#include <sensor_msgs/PointCloud2.h>

#include <filesystem>
#include <optional>

namespace xgc_scene {

// The selected sealed asset is resolved by inputs(). Read its original XYZ
// attachment once; the message retains the original float32 bytes unchanged.
sensor_msgs::PointCloud2 fixedPointCloud(const Json& selected);

// The frozen Session's sceneAsset.pointCloud is the only source of the saved
// cloud's metadata: manifest.json is not opened. Returns nothing when the
// asset has no saved cloud. The attachment is read from asset_directory, the
// Run's runtime copy of the sealed asset.
std::optional<sensor_msgs::PointCloud2> frozenFixedCloud(const Json& session,
                                                         const std::filesystem::path& asset_directory);

}  // namespace xgc_scene
