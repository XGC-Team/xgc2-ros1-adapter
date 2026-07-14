#pragma once

#include <signal.h>

namespace xgc_px4_multirotor_ros1_adapter {

// Records SIGINT and SIGTERM without invoking ROS or gRPC from the asynchronous
// signal context. The main thread can then perform the normal orderly shutdown.
class ShutdownSignalHandler final {
public:
  ShutdownSignalHandler();
  ~ShutdownSignalHandler();

  ShutdownSignalHandler(const ShutdownSignalHandler &) = delete;
  ShutdownSignalHandler &operator=(const ShutdownSignalHandler &) = delete;

  bool requested() const noexcept;
  int signalNumber() const noexcept;

private:
  struct sigaction previous_sigint_ {};
  struct sigaction previous_sigterm_ {};
  bool sigint_installed_ = false;
  bool sigterm_installed_ = false;
};

} // namespace xgc_px4_multirotor_ros1_adapter
