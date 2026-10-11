#pragma once

#include <chrono>
#include <algorithm>
#include <array>
#include <iostream>
#include <random>
#include <string>

namespace xgc_scene {

// Used only by the two peer owners, never by ROS callbacks or scan workers.
class PeerBackoff {
 public:
  using Clock = std::chrono::steady_clock;
  std::chrono::milliseconds failed(const std::string& peer, const std::string& reason) {
    const auto now = Clock::now();
    if (!outage_) {
      outage_ = true;
      summary_ = now;
      std::cerr << "scene adapter: " << peer << " waiting: " << reason << '\n';
    } else if (now - summary_ >= std::chrono::seconds(10)) {
      std::cerr << "scene adapter: " << peer << " waiting; suppressed " << suppressed_ << " attempts\n";
      summary_ = now;
      suppressed_ = 0;
    } else ++suppressed_;
    failures_[next_] = now;
    next_ = (next_ + 1) % failures_.size();
    attempts_ = std::min(attempts_ + 1, static_cast<int>(failures_.size()));
    auto delay = std::chrono::milliseconds(std::uniform_int_distribution<int>(0, cap_)(random_));
    // Round a zero draw up to one millisecond; never spin or sleep(0).
    delay = std::max(delay, std::chrono::milliseconds(1));
    cap_ = std::min(cap_ * 2, 10000);
    // Full jitter can draw repeated near-zero intervals. Bound that tail too.
    if (attempts_ == static_cast<int>(failures_.size()))
      delay = std::max(delay, std::chrono::duration_cast<std::chrono::milliseconds>(failures_[next_] + std::chrono::seconds(60) - now) + std::chrono::milliseconds(1));
    return delay;
  }
  void success() { cap_ = 250; outage_ = false; attempts_ = suppressed_ = 0; next_ = 0; }
 private:
  std::mt19937 random_{std::random_device{}()};
  int cap_ = 250, attempts_ = 0, suppressed_ = 0;
  bool outage_ = false;
  Clock::time_point summary_;
  std::array<Clock::time_point, 15> failures_;
  std::size_t next_ = 0;
};
}  // namespace xgc_scene
