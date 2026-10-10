#include "xgc_mecanum_ugv_ros1_adapter/shutdown_signal.hpp"

#include <cerrno>
#include <system_error>

namespace xgc_mecanum_ugv_ros1_adapter {
namespace {

volatile sig_atomic_t g_shutdown_signal = 0;

void recordShutdownSignal(int signal_number) noexcept {
  g_shutdown_signal = signal_number;
}

void installSignalHandler(int signal_number,
                          struct sigaction *previous_action) {
  struct sigaction action {};
  action.sa_handler = recordShutdownSignal;
  sigemptyset(&action.sa_mask);
  action.sa_flags = 0;
  if (sigaction(signal_number, &action, previous_action) != 0) {
    throw std::system_error(
        errno, std::generic_category(),
        "failed to install adapter shutdown signal handler");
  }
}

} // namespace

ShutdownSignalHandler::ShutdownSignalHandler() {
  g_shutdown_signal = 0;
  installSignalHandler(SIGINT, &previous_sigint_);
  sigint_installed_ = true;
  try {
    installSignalHandler(SIGTERM, &previous_sigterm_);
    sigterm_installed_ = true;
  } catch (...) {
    sigaction(SIGINT, &previous_sigint_, nullptr);
    sigint_installed_ = false;
    throw;
  }
}

ShutdownSignalHandler::~ShutdownSignalHandler() {
  if (sigterm_installed_) {
    sigaction(SIGTERM, &previous_sigterm_, nullptr);
  }
  if (sigint_installed_) {
    sigaction(SIGINT, &previous_sigint_, nullptr);
  }
}

bool ShutdownSignalHandler::requested() const noexcept {
  return g_shutdown_signal != 0;
}

int ShutdownSignalHandler::signalNumber() const noexcept {
  return g_shutdown_signal;
}

} // namespace xgc_mecanum_ugv_ros1_adapter
