#include "sim_fcu_rpc.hpp"

#include <cassert>
#include <future>
#include <iostream>
#include <thread>

using xgc_sim_fcu::Rpc;
using namespace std::chrono_literals;

xgc_fcu_request_v2 arm(uint32_t flags = 0) {
  xgc_fcu_request_v2 request{};
  request.stamp = 12.5;
  request.kind = 1;
  request.arm = 1;
  request.flags = flags;
  return request;
}

void ready(Rpc& rpc) {
  rpc.open();
  rpc.output_open(true);
  rpc.state(10, true);
}

xgc_fcu_request_v2 queued(Rpc& rpc) {
  const auto deadline = Rpc::Clock::now() + 200ms;
  while (Rpc::Clock::now() < deadline) {
    const auto requests = rpc.take_requests();
    if (!requests.empty()) {
      assert(requests.size() == 1);
      return requests.front();
    }
    std::this_thread::sleep_for(1ms);
  }
  assert(false && "request was not enqueued");
  return {};
}

xgc_fcu_result_v1 ack(const xgc_fcu_request_v2& request, uint32_t result, uint32_t robot = 5) {
  xgc_fcu_result_v1 response{};
  response.stamp = 13;
  response.request_stamp = request.stamp;
  response.request_id = request.request_id;
  response.robot_index = robot;
  response.kind = request.kind;
  response.result = result;
  return response;
}

int main() {
  {
    Rpc rpc(5, 200ms, 500ms);
    ready(rpc);
    auto call = std::async(std::launch::async, [&] { return rpc.call(arm(), true); });
    const auto request = queued(rpc);
    assert(request.request_id & (uint64_t{1} << 63));
    assert(call.wait_for(0ms) == std::future_status::timeout);
    assert(!rpc.result(ack(request, 0)));  // Mere enqueue cannot accept an ACK.
    rpc.published(request.request_id, true);
    auto wrong = ack(request, 0);
    wrong.request_id &= ~(uint64_t{1} << 63);  // Native controller ID.
    assert(!rpc.result(wrong));
    wrong = ack(request, 0, 4);
    assert(!rpc.result(wrong));
    wrong = ack(request, 0);
    wrong.kind = 2;
    assert(!rpc.result(wrong));
    wrong = ack(request, 0);
    wrong.request_stamp += 1;
    assert(!rpc.result(wrong));
    wrong = ack(request, 0);
    wrong.stamp = -1;
    assert(!rpc.result(wrong));
    wrong = ack(request, 0);
    wrong.reserved = 1;
    assert(!rpc.result(wrong));
    assert(call.wait_for(0ms) == std::future_status::timeout);
    assert(rpc.result(ack(request, 0)));
    const auto reply = call.get();
    assert(reply.sent && reply.has_result && reply.result == 0);
  }
  for (const auto code : {1u, 2u, 3u, 4u}) {
    Rpc rpc(5, 200ms, 500ms);
    ready(rpc);
    auto call = std::async(std::launch::async, [&] { return rpc.call(arm(1), true); });
    const auto request = queued(rpc);
    assert(request.flags == 1);
    rpc.published(request.request_id, true);
    assert(rpc.result(ack(request, code)));
    const auto reply = call.get();
    assert(reply.sent && reply.has_result && reply.result == code);
  }
  {
    Rpc rpc(5, 100ms, 500ms);
    ready(rpc);
    auto request = arm();
    request.kind = 2;
    auto call = std::async(std::launch::async, [&] { return rpc.call(request, false); });
    const auto sent = queued(rpc);
    rpc.published(sent.request_id, true);
    const auto reply = call.get();
    assert(reply.sent && !reply.has_result);  // mode_sent is not model acceptance.
  }
  {
    Rpc rpc(5, 30ms, 500ms);
    ready(rpc);
    const auto started = Rpc::Clock::now();
    const auto reply = rpc.call(arm(), true);
    assert(!reply.sent && !reply.has_result && reply.result == 4);
    assert(Rpc::Clock::now() - started < 250ms);
    assert(rpc.take_requests().empty());  // An expired unissued request stays unissued.
  }
  {
    Rpc rpc(5, 30ms, 500ms);
    ready(rpc);
    auto call = std::async(std::launch::async, [&] { return rpc.call(arm(), true); });
    const auto request = queued(rpc);
    rpc.published(request.request_id, true);
    const auto reply = call.get();
    assert(reply.sent && !reply.has_result && reply.result == 4);
    assert(!rpc.result(ack(request, 0)));  // Late executed results cannot credit a later RPC.
  }
  {
    Rpc rpc(5, 100ms, 40ms);
    rpc.open();
    rpc.output_open(true);
    rpc.state(10, true, Rpc::Clock::now() - 60ms);
    rpc.state(10, true);  // Repeated source stamp must not refresh arrival time.
    assert(!rpc.call(arm(), true).sent);
    rpc.state(9, true);
    assert(!rpc.call(arm(), true).sent);
  }
  for (const bool stop : {false, true}) {
    Rpc rpc(5, 1000ms, 500ms);
    ready(rpc);
    auto call = std::async(std::launch::async, [&] { return rpc.call(arm(), true); });
    const auto request = queued(rpc);
    rpc.published(request.request_id, true);
    const auto started = Rpc::Clock::now();
    if (stop) rpc.close(); else rpc.output_open(false);
    assert(call.wait_for(100ms) == std::future_status::ready);
    assert(!call.get().has_result);
    assert(Rpc::Clock::now() - started < 150ms);
    assert(!rpc.result(ack(request, 0)));
    rpc.open();
    rpc.output_open(true);
    rpc.state(10, true);  // Reactivation must not treat the pre-stop stamp as new.
    assert(!rpc.call(arm(), true).sent);
  }
  {
    Rpc rpc(5, 100ms, 500ms);
    ready(rpc);
    auto call = std::async(std::launch::async, [&] { return rpc.call(arm(), true); });
    const auto request = queued(rpc);
    rpc.published(request.request_id, false);
    const auto reply = call.get();
    assert(!reply.sent && !reply.has_result && reply.result == 4);
  }
  std::cout << "sim FCU RPC correlation, force, freshness, bounded timeout and stop checks passed\n";
}
