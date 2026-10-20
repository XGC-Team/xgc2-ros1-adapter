#pragma once

#include <chrono>
#include <functional>
#include <mutex>

namespace xgc2_ros1_robot_adapter {

// How long the Core has not been subscribed to this server's status stream.
// A source-side adapter keeps driving the robot after the last operator
// intent; without this, a Core that is gone (crash, link loss, closed
// Session) would leave the robot at its last velocity. The server reports the
// silence to its robots, which release a motion intent that outlived the Core.
//
// Silence is measured from the moment the last stream ended (or from the
// moment the server started when Core never subscribed). The clock is
// injectable so tests move time without sleeping.
class CoreSilence {
public:
  using Clock = std::function<std::chrono::steady_clock::time_point()>;

  explicit CoreSilence(Clock clock = [] { return std::chrono::steady_clock::now(); })
      : clock_(std::move(clock)), since_(clock_()) {}

  // A status stream started / ended. Streams may overlap (Core renews its
  // subscription before the old one ends).
  void Opened() {
    std::lock_guard<std::mutex> lock(mutex_);
    ++streams_;
  }
  void Closed() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (streams_ > 0 && --streams_ == 0)
      since_ = clock_();
  }

  // Zero while a stream is open; otherwise the time since the last one ended.
  std::chrono::milliseconds SilentFor() const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (streams_ > 0)
      return std::chrono::milliseconds::zero();
    return std::chrono::duration_cast<std::chrono::milliseconds>(clock_() - since_);
  }

private:
  const Clock clock_;
  mutable std::mutex mutex_;
  std::size_t streams_ = 0;
  std::chrono::steady_clock::time_point since_;
};

} // namespace xgc2_ros1_robot_adapter
