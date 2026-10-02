#pragma once
#include "sim_fcu_rpc.hpp"

namespace xgc_sim_provider {

struct State {
  uint64_t generation{0};
  bool enabled{false};
};
struct Reply {
  bool received{false};
  xgc_sim_provider_result_v1 result{};
};

// Lifecycle requests do not require an online FCU: provider start is what
// creates that online state. Only matching executed results change State.
class Rpc {
 public:
  using Clock = std::chrono::steady_clock;
  explicit Rpc(uint32_t index, std::chrono::milliseconds timeout = std::chrono::milliseconds(1500))
      : index_(index), timeout_(timeout) {
    if (index >= 6 || timeout.count() < 1 || timeout.count() > 5000)
      throw std::invalid_argument("invalid provider RPC index or timeout");
  }
  void open() { std::lock_guard<std::mutex> lock(mutex_); active_ = true; }
  void close() {
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = false;
    for (auto& p : pending_) p.second->done = true;
    queued_.clear();
    changed_.notify_all();
  }
  State state() const { std::lock_guard<std::mutex> lock(mutex_); return state_; }
  Reply call(xgc_sim_provider_request_v1 request) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!active_ || !std::isfinite(request.stamp) || request.stamp < 0 ||
        request.action > 2 || pending_.size() >= 16) return {};
    request.robot_index = index_;
    request.request_id = xgc_sim_fcu::next_rpc_id();
    if (!request.request_id) return {};
    auto p = std::make_shared<Pending>();
    p->request = request;
    p->deadline = Clock::now() + timeout_;
    pending_[request.request_id] = p;
    queued_.push_back(p);
    changed_.wait_until(lock, p->deadline, [&] { return p->done; });
    p->done = true;
    pending_.erase(request.request_id);
    return p->reply;
  }
  std::vector<xgc_sim_provider_request_v1> take_requests() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<xgc_sim_provider_request_v1> requests;
    while (!queued_.empty()) {
      auto p = queued_.front(); queued_.pop_front();
      if (p->done || !active_ || Clock::now() >= p->deadline) { p->done = true; continue; }
      requests.push_back(p->request);
    }
    changed_.notify_all();
    return requests;
  }
  void published(uint64_t id, bool ok) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto p = pending_.find(id);
    if (p == pending_.end() || p->second->done) return;
    p->second->sent = ok;
    if (!ok) { p->second->done = true; changed_.notify_all(); }
  }
  bool result(const xgc_sim_provider_result_v1& result) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_ || result.robot_index != index_ || !std::isfinite(result.stamp) || result.stamp < 0 ||
        result.accepted > 1 || result.enabled > 1 || result.reason > 2) return false;
    auto found = pending_.find(result.request_id);
    if (found == pending_.end()) return false;
    auto& p = *found->second;
    if (p.done || !p.sent || Clock::now() >= p.deadline) return false;
    if (result.accepted && ((p.request.action == 1 && (!result.enabled || !result.generation)) ||
                           (p.request.action == 2 && (result.enabled || result.generation != p.request.generation))))
      return false;
    p.reply.received = true;
    p.reply.result = result;
    p.done = true;
    // A rejected stale request may report the newer model generation. Never
    // move the edge backwards or close that newer generation.
    if (result.generation >= state_.generation && result.stamp >= last_stamp_) {
      state_ = {result.generation, result.enabled != 0};
      last_stamp_ = result.stamp;
    }
    changed_.notify_all();
    return true;
  }
 private:
  struct Pending {
    xgc_sim_provider_request_v1 request{};
    Clock::time_point deadline;
    bool sent{false}, done{false};
    Reply reply;
  };
  uint32_t index_;
  std::chrono::milliseconds timeout_;
  mutable std::mutex mutex_;
  std::condition_variable changed_;
  bool active_{false};
  State state_;
  double last_stamp_{-1};
  std::deque<std::shared_ptr<Pending>> queued_;
  std::unordered_map<uint64_t, std::shared_ptr<Pending>> pending_;
};
}  // namespace xgc_sim_provider
