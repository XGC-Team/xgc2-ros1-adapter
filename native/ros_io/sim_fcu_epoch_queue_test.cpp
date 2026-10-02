// Deterministic real ROS callback-queue regression. The first callback is
// blocked independently of the RPC helper; an old disarm is then queued.
// Provider replies are staged ABI inputs, not changes to the ROS facade.
#include "ros_io.cpp"
#include <cassert>
#include <future>
#include <iostream>

using namespace std::chrono_literals;

struct Block : ros::CallbackInterface {
  std::mutex mutex;
  std::condition_variable cv;
  bool entered{false}, released{false};
  CallResult call() override {
    std::unique_lock<std::mutex> lock(mutex);
    entered = true; cv.notify_all();
    cv.wait(lock, [&] { return released; });
    return Success;
  }
  void wait() { std::unique_lock<std::mutex> lock(mutex); assert(cv.wait_for(lock, 1s, [&] { return entered; })); }
  void release() { std::lock_guard<std::mutex> lock(mutex); released = true; cv.notify_all(); }
};

struct Boundary {
  std::array<std::deque<std::vector<uint8_t>>, kPortCount> staged;
  std::deque<xgc_sim_provider_result_v1> provider_results;
  std::deque<xgc_fcu_result_v1> fcu_results;
  std::vector<uint8_t> popped;
  uint64_t generation{0}, disarms{0};
  bool enabled{false}, armed{true};
  double stamp{1};
  const std::thread::id owner{std::this_thread::get_id()};
  xgc_host_api api{};
  Boundary() {
    api.abi_version = XGC_RT_ABI_VERSION; api.abi_minor = XGC_RT_ABI_MINOR; api.host = this;
    api.now = [](void*) { return int64_t{0}; };
    api.log = [](void*, xgc_log_level, const char*) {};
    api.next = [](void* p, uint32_t port, xgc_sample_view* view) {
      auto& b = *static_cast<Boundary*>(p); assert(b.owner == std::this_thread::get_id());
      if (b.staged[port].empty()) return XGC_ERR_AGAIN;
      b.popped = std::move(b.staged[port].front()); b.staged[port].pop_front();
      *view = {}; view->data = b.popped.data(); view->len = b.popped.size(); return XGC_OK;
    };
    api.publish = [](void* p, uint32_t port, uint64_t, const uint8_t* data, uint32_t size) {
      auto& b = *static_cast<Boundary*>(p); assert(b.owner == std::this_thread::get_id());
      if (port == kSimProviderRequest) {
        assert(size == sizeof(xgc_sim_provider_request_v1));
        xgc_sim_provider_request_v1 q; std::memcpy(&q, data, sizeof q);
        xgc_sim_provider_result_v1 r{}; r.stamp = b.stamp; r.request_id = q.request_id;
        r.robot_index = q.robot_index;
        if (q.action == 0) r.accepted = 1;
        else if (q.action == 1 && ((!b.enabled && q.generation == b.generation) ||
                 (b.enabled && (q.generation == b.generation || q.generation + 1 == b.generation)))) {
          if (!b.enabled) { ++b.generation; b.enabled = true; b.armed = true; }
          r.accepted = 1;
        } else if (q.action == 2 && q.generation == b.generation) { b.enabled = false; r.accepted = 1; }
        else r.reason = 1;
        r.enabled = b.enabled; r.generation = b.generation; b.provider_results.push_back(r);
      } else if (port == kSimFcuRequest) {
        xgc_fcu_request_v2 q; assert(size == sizeof q); std::memcpy(&q, data, sizeof q);
        if (q.kind == 1 && !q.arm) { ++b.disarms; b.armed = false; }
        xgc_fcu_result_v1 r{}; r.stamp = b.stamp; r.request_stamp = q.stamp;
        r.request_id = q.request_id; r.kind = q.kind; b.fcu_results.push_back(r);
      }
      return XGC_OK;
    };
  }
  template <class T> void input(Port port, const T& value) {
    auto* bytes = reinterpret_cast<const uint8_t*>(&value);
    staged[port].emplace_back(bytes, bytes + sizeof value);
  }
  void tick(RosIo& edge) {
    stamp += 0.01;
    xgc_fcu_state_v1 state{}; state.stamp = stamp; state.connected = enabled; state.armed = armed;
    input(kSimFcuState, state);
    for (const auto& r : provider_results) input(kSimProviderResult, r);
    for (const auto& r : fcu_results) input(kSimFcuResult, r);
    provider_results.clear(); fcu_results.clear();
    xgc_step_ctx ctx{}; assert(edge.step(&ctx) == XGC_OK);
    std::this_thread::sleep_for(1ms);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "fcu_epoch_queue_test", ros::init_options::NoSigintHandler);
  Boundary boundary;
  auto* edge = static_cast<RosIo*>(create(&boundary.api)); assert(edge);
  assert(configure(edge, "node_name=\"fcu_epoch_queue_test\"\nslice_ms=0.001\n"
      "sim_fcu_state_topic=\"/private_epoch/state\"\nsim_fcu_request_topic=\"/private_epoch/mavros\"\n"
      "sim_provider_service=\"/private_epoch/provider\"\n") == XGC_OK);
  assert(edge->activate() == XGC_OK);
  ros::NodeHandle nh;
  auto provider = nh.serviceClient<sss_sim_env::SetProvider>("/private_epoch/provider");
  auto arming = nh.serviceClient<mavros_msgs::CommandBool>("/private_epoch/mavros/cmd/arming");
  assert(provider.waitForExistence(ros::Duration(2)));
  auto wait = [&](const auto& condition) {
    const auto deadline = std::chrono::steady_clock::now() + 1s;
    while (!condition()) { assert(std::chrono::steady_clock::now() < deadline); boundary.tick(*edge); }
  };
  auto lifecycle = [&](uint8_t action, uint64_t expected) {
    sss_sim_env::SetProvider call; call.request.action = action; call.request.generation = expected;
    auto future = std::async(std::launch::async, [&] { return provider.call(call); });
    wait([&] { return future.wait_for(0ms) == std::future_status::ready; });
    assert(future.get() && call.response.accepted); return call.response;
  };
  assert(lifecycle(0, 0).generation == 0 && !boundary.enabled);
  assert(lifecycle(1, 0).generation == 1);
  boundary.tick(*edge);
  const auto old_epoch = edge->sim_rpc->epoch();
  auto loop = shared_sim_service_loop();
  boost::shared_ptr<Block> block(new Block);
  loop->queue.addCallback(block); block->wait();
  mavros_msgs::CommandBool old_disarm; old_disarm.request.value = false;
  auto old_call = std::async(std::launch::async, [&] { return arming.call(old_disarm); });
  wait([&] { return !loop->queue.isEmpty(); });  // Old service callback really entered the ROS queue.
  assert(lifecycle(2, 1).generation == 1);
  assert(lifecycle(1, 1).generation == 2);
  boundary.tick(*edge); assert(boundary.enabled && boundary.armed);
  assert(edge->sim_rpc->epoch() != old_epoch);
  block->release();
  wait([&] { return old_call.wait_for(0ms) == std::future_status::ready; });
  const bool transport = old_call.get();
  assert(!transport || (!old_disarm.response.success && old_disarm.response.result == 4));
  for (int i = 0; i < 20; ++i) boundary.tick(*edge);
  assert(boundary.disarms == 0 && boundary.armed && boundary.generation == 2);
  // Also cover check-then-enqueue: a callback with a captured retired epoch
  // cannot enter the reopened helper even when gen2 FCU feedback is fresh.
  mavros_msgs::CommandBool::Response retired;
  RosIo::on_sim_arming(edge->sim_rpc, old_epoch, old_disarm.request, retired);
  assert(!retired.success && retired.result == 4 && edge->sim_rpc->take_requests().empty());
  mavros_msgs::CommandBool current; current.request.value = false;
  auto valid = std::async(std::launch::async, [&] { return arming.call(current); });
  wait([&] { return valid.wait_for(0ms) == std::future_status::ready; });
  assert(valid.get() && current.response.success && boundary.disarms == 1);
  edge->shutdown(); destroy(edge);
  std::cout << "{\"result\":\"passed\",\"queued_old_generation_disarms\":0,"
               "\"current_generation_disarms\":1,\"generation\":2,\"captured_epoch_rejected\":true}\n";
}
