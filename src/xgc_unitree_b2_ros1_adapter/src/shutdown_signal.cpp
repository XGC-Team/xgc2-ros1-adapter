#include "xgc_unitree_b2_ros1_adapter/shutdown_signal.hpp"
#include <csignal>
namespace xgc_unitree_b2_ros1_adapter {
std::atomic<bool> ShutdownSignalHandler::requested_{false};
ShutdownSignalHandler::ShutdownSignalHandler() {
  requested_.store(false); std::signal(SIGINT, Handle); std::signal(SIGTERM, Handle);
}
ShutdownSignalHandler::~ShutdownSignalHandler() = default;
void ShutdownSignalHandler::Handle(int) { requested_.store(true); }
bool ShutdownSignalHandler::requested() const noexcept { return requested_.load(); }
}  // namespace xgc_unitree_b2_ros1_adapter
