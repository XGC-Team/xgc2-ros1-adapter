#pragma once

// The ROS service thread enqueues and waits here. Only the Host step takes
// requests, publishes them and supplies executed results. No Host API is
// called from this helper or from a ROS service callback.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include "xgc_schemas_v1.h"

namespace xgc_sim_fcu {

inline uint64_t next_rpc_id() {
  static std::atomic<uint64_t> next{1};
  constexpr uint64_t high_bit = uint64_t{1} << 63;
  const uint64_t serial = next.fetch_add(1, std::memory_order_relaxed);
  return serial > 0 && serial < high_bit ? high_bit | serial : 0;
}

struct Reply {
  bool sent{false};
  bool has_result{false};
  uint32_t result{4};
};

class Rpc {
 public:
  using Clock = std::chrono::steady_clock;
  using Duration = std::chrono::milliseconds;

  Rpc(uint32_t robot_index, Duration timeout = Duration(1000), Duration freshness = Duration(500))
      : robot_index_(robot_index), timeout_(timeout), freshness_(freshness) {
    if (robot_index >= 6 || timeout.count() < 1 || timeout.count() > 5000 ||
        freshness.count() < 1 || freshness.count() > 5000)
      throw std::invalid_argument("invalid simulated FCU RPC index or bounded deadlines");
  }

  void open() {
    std::lock_guard<std::mutex> lock(mutex_);
    ++epoch_;
    active_ = true;
    output_open_ = false;
    have_state_ = false;
    // Keep last_stamp_: replaying the pre-deactivate sample is not a new arrival.
  }

  void close() {
    std::lock_guard<std::mutex> lock(mutex_);
    ++epoch_;  // Fence queued callbacks before closing/waking old pending calls.
    active_ = false;
    output_open_ = false;
    have_state_ = false;
    fail_pending();
  }

  void output_open(bool open) {
    std::lock_guard<std::mutex> lock(mutex_);
    output_open_ = open && active_;
    if (!output_open_) {
      have_state_ = false;
      fail_pending();
    }
  }

  void state(double stamp, bool connected, Clock::time_point arrived = Clock::now()) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_ || !output_open_ || !std::isfinite(stamp) || stamp < 0 ||
        (have_stamp_ && stamp <= last_stamp_)) return;
    have_stamp_ = true;
    last_stamp_ = stamp;
    have_state_ = true;
    connected_ = connected;
    arrived_ = arrived;
    if (!connected) fail_pending();
  }

  uint64_t epoch() const { std::lock_guard<std::mutex> lock(mutex_); return epoch_; }

  Reply call(xgc_fcu_request_v2 request, bool need_result, uint64_t expected_epoch) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (expected_epoch != epoch_ || !ready(Clock::now()) || !std::isfinite(request.stamp) || request.stamp < 0 ||
        (request.kind != 1 && request.kind != 2) || pending_.size() >= 16) return {};
    request.request_id = next_rpc_id();  // Native controller IDs use the low half.
    if (request.request_id == 0) return {};
    auto pending = std::make_shared<Pending>();
    pending->request = request;
    pending->need_result = need_result;
    pending->deadline = Clock::now() + timeout_;
    pending_[request.request_id] = pending;
    queued_.push_back(pending);
    while (!pending->done) {
      const auto now = Clock::now();
      if (!ready(now) || now >= pending->deadline) {
        pending->done = true;
        break;
      }
      changed_.wait_until(lock, std::min(pending->deadline, arrived_ + freshness_));
    }
    pending_.erase(request.request_id);
    return pending->reply;
  }

  Reply call(xgc_fcu_request_v2 request, bool need_result) { return call(request, need_result, epoch()); }

  // This never waits for an ACK. next() in the Host only sees the snapshot
  // staged at step entry; the result is consumed by a subsequent step.
  std::vector<xgc_fcu_request_v2> take_requests() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<xgc_fcu_request_v2> requests;
    while (!queued_.empty()) {
      const auto pending = queued_.front();
      queued_.pop_front();
      if (pending->done) continue;
      if (!ready(Clock::now()) || Clock::now() >= pending->deadline) {
        pending->done = true;
        changed_.notify_all();
        continue;
      }
      requests.push_back(pending->request);
    }
    return requests;
  }

  bool can_publish(uint64_t id) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = pending_.find(id);
    return found != pending_.end() && !found->second->done && ready(Clock::now()) &&
           Clock::now() < found->second->deadline;
  }

  void published(uint64_t id, bool ok) {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto found = pending_.find(id);
    if (found == pending_.end() || found->second->done) return;
    auto& pending = *found->second;
    pending.reply.sent = ok;
    if (!ok || !pending.need_result) {
      pending.done = true;
      // SetMode only reports delivery. It has no execution result yet.
      changed_.notify_all();
    }
  }

  bool result(const xgc_fcu_result_v1& result) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!ready(Clock::now()) || result.robot_index != robot_index_ || result.reserved != 0 ||
        !std::isfinite(result.stamp) || result.stamp < 0 || !std::isfinite(result.request_stamp) ||
        result.request_stamp < 0 || result.result > 4) return false;
    const auto found = pending_.find(result.request_id);
    if (found == pending_.end()) return false;
    auto& pending = *found->second;
    if (pending.done || !pending.reply.sent || Clock::now() >= pending.deadline ||
        result.kind != pending.request.kind || result.request_stamp != pending.request.stamp) return false;
    pending.reply.has_result = true;
    pending.reply.result = result.result;
    pending.done = true;
    changed_.notify_all();
    return true;
  }

 private:
  struct Pending {
    xgc_fcu_request_v2 request{};
    Clock::time_point deadline;
    bool need_result{true};
    bool done{false};
    Reply reply;
  };

  bool ready(Clock::time_point now) const {
    return active_ && output_open_ && have_state_ && connected_ && now >= arrived_ &&
           now - arrived_ < freshness_;
  }

  void fail_pending() {
    for (auto& entry : pending_) entry.second->done = true;
    queued_.clear();
    changed_.notify_all();
  }

  uint32_t robot_index_;
  Duration timeout_, freshness_;
  mutable std::mutex mutex_;
  uint64_t epoch_{0};
  std::condition_variable changed_;
  bool active_{false}, output_open_{false}, have_state_{false}, connected_{false}, have_stamp_{false};
  double last_stamp_{0};
  Clock::time_point arrived_;
  std::deque<std::shared_ptr<Pending>> queued_;
  std::unordered_map<uint64_t, std::shared_ptr<Pending>> pending_;
};

}  // namespace xgc_sim_fcu
