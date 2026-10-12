#pragma once

#include "xgc_scene/common.hpp"
#include <sensor_msgs/PointCloud2.h>

namespace xgc_scene {

// The selected sealed asset is resolved by inputs(). Read its original XYZ
// attachment once; the message retains the original float32 bytes unchanged.
sensor_msgs::PointCloud2 fixedPointCloud(const Json& selected);

}  // namespace xgc_scene
