#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "xgc2_ros1_robot_adapter/zenoh_localization_config.hpp"

namespace xgc2_ros1_robot_adapter {

std::string ZenohLocalizationKey(const ZenohLocalizationBinding &binding,
                                 const char *kind);

// Shared, in-process Zenoh pose receiver. It owns one exact-key subscriber and
// invokes the existing Adapter callback on the Zenoh callback thread. It does
// not publish ROS and does not create a second localization protocol.
class ZenohLocalizationInput {
public:
  using PoseCallback = std::function<void(const ZenohPoseSample &)>;

  static std::unique_ptr<ZenohLocalizationInput>
  Open(const ZenohLocalizationBinding &binding, PoseCallback callback,
       std::string *error);

  ~ZenohLocalizationInput();
  ZenohLocalizationInput(const ZenohLocalizationInput &) = delete;
  ZenohLocalizationInput &operator=(const ZenohLocalizationInput &) = delete;

  // Idempotent. Stops dispatching and drops the subscriber/session.
  void Stop();

private:
  struct Impl;
  explicit ZenohLocalizationInput(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};

} // namespace xgc2_ros1_robot_adapter
