#pragma once
#include "xgc_scene/store.hpp"
#include <algorithm>
#include <atomic>
#include <chrono>
#include <functional>
#include <future>
#include <memory>
#include <stdexcept>

namespace xgc_scene {
struct CommandContext {
  using Clock = std::chrono::steady_clock;
  Clock::time_point deadline = Clock::time_point::max();
  std::function<bool()> cancelled;
  bool expired() const { return Clock::now() >= deadline || (cancelled && cancelled()); }
};
class CommandExpired : public std::runtime_error {
 public:
  CommandExpired() : std::runtime_error("Scene command was cancelled or expired before application") {}
};

// Cancellation wins only before the lane claims the command. A running command
// keeps its domain request identity; an elapsed caller budget has unknown outcome.
inline Json dispatchCommand(std::function<void(std::function<void()>)> post,
                            std::function<Json()> command, CommandContext context,
                            const std::atomic<bool>& stopping) {
  struct State {
    std::atomic<int> phase{0}; // queued, running, completed, cancelled
    std::promise<Json> result;
  };
  auto state = std::make_shared<State>();
  auto future = state->result.get_future();
  post([state, command = std::move(command), context, stop = &stopping] {
    int queued = 0;
    if (!state->phase.compare_exchange_strong(queued, 1)) return;
    try {
      if (stop->load() || context.expired()) throw CommandExpired();
      state->result.set_value(command());
    } catch (...) { state->result.set_exception(std::current_exception()); }
    state->phase.store(2);
  });
  while (future.wait_until(std::min(context.deadline, CommandContext::Clock::now() + std::chrono::milliseconds(100)))
         != std::future_status::ready) {
    if (stopping.load() || context.expired()) {
      int queued = 0;
      if (state->phase.compare_exchange_strong(queued, 3)) throw CommandExpired();
      if (future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) break;
      throw std::runtime_error("Scene command completion deadline elapsed after application started");
    }
  }
  return future.get();
}
} // namespace xgc_scene
