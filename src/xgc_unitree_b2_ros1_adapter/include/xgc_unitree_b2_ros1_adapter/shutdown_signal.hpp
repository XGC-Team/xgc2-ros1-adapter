#pragma once
#include <atomic>
namespace xgc_unitree_b2_ros1_adapter {
class ShutdownSignalHandler {
 public:
  ShutdownSignalHandler();
  ~ShutdownSignalHandler();
  bool requested() const noexcept;
 private:
  static void Handle(int signal);
  static std::atomic<bool> requested_;
};
}  // namespace xgc_unitree_b2_ros1_adapter
