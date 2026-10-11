#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <random>

namespace xgc2_ros1_robot_adapter {

// One owner per master. Only failures back off; a successful probe resets the
// outage. The one-millisecond resolution avoids a zero-duration retry loop.
class MasterRetry {
public:
  explicit MasterRetry(std::chrono::milliseconds initial =
                           std::chrono::milliseconds(250))
      : initial_(initial), ceiling_(initial), random_(std::random_device{}()) {}

  std::chrono::milliseconds failed() {
    const auto now = std::chrono::steady_clock::now();
    ++attempts_;
    if (attempts_ == 1) {
      std::clog << "[ros-master] unavailable; outage_attempts=1\n";
      last_log_ = now;
    } else {
      ++suppressed_;
      if (now - last_log_ >= std::chrono::seconds(10)) {
        std::clog << "[ros-master] still unavailable; suppressed_attempts="
                  << suppressed_ << "; outage_attempts=" << attempts_ << '\n';
        suppressed_ = 0;
        last_log_ = now;
      }
    }
    const auto delay = std::chrono::milliseconds(
        std::uniform_int_distribution<std::int64_t>(1, ceiling_.count())(random_));
    ceiling_ = std::min(ceiling_ * 2, std::chrono::milliseconds(10000));
    return delay;
  }

  void reset() {
    ceiling_ = initial_;
    attempts_ = suppressed_ = 0;
  }

private:
  const std::chrono::milliseconds initial_;
  std::chrono::milliseconds ceiling_;
  std::mt19937 random_;
  std::uint64_t attempts_ = 0, suppressed_ = 0;
  std::chrono::steady_clock::time_point last_log_;
};
} // namespace xgc2_ros1_robot_adapter
