#include "sim_provider_rpc.hpp"
#include <cassert>
#include <future>
#include <iostream>
#include <thread>
using namespace std::chrono_literals;
using xgc_sim_provider::Rpc;
using xgc_sim_provider::Groups;

int main() {
  auto a0 = std::make_shared<Rpc>(0, 500ms), a1 = std::make_shared<Rpc>(1, 500ms);
  auto b0 = std::make_shared<Rpc>(0, 500ms), w0 = std::make_shared<Rpc>(0, 500ms);
  auto& groups = Groups::instance();
  for (auto& rpc : {a0, a1, b0, w0}) rpc->open();
  groups.add("worldA/batch0", 0, a0); groups.add("worldA/batch0", 1, a1);
  groups.add("worldA/batch1", 0, b0); groups.add("worldB/batch0", 0, w0);
  xgc_sim_provider_request_v1 query{}; query.stamp = 1; query.action = 0;
  auto c0 = std::async(std::launch::async, [&] { return a0->call(query); });
  auto c1 = std::async(std::launch::async, [&] { return a1->call(query); });
  auto cb = std::async(std::launch::async, [&] { return b0->call(query); });
  auto cw = std::async(std::launch::async, [&] { return w0->call(query); });
  std::vector<Groups::Work> work;
  const auto deadline = std::chrono::steady_clock::now() + 200ms;
  while (work.size() < 2) {
    auto next = groups.take("worldA/batch0"); work.insert(work.end(), next.begin(), next.end());
    assert(std::chrono::steady_clock::now() < deadline); std::this_thread::sleep_for(1ms);
  }
  assert(work[0].request.request_id != work[1].request.request_id);
  for (auto& item : work) {
    assert(item.rpc == a0 || item.rpc == a1);
    item.rpc->published(item.request.request_id, true);
    xgc_sim_provider_result_v1 result{}; result.stamp = 2; result.request_id = item.request.request_id;
    result.robot_index = item.request.robot_index; result.accepted = 1;
    auto wrong = result; wrong.robot_index = 5; assert(!item.rpc->result(wrong));
    assert(item.rpc->result(result));
  }
  assert(c0.get().received && c1.get().received);
  assert(cb.wait_for(0ms) == std::future_status::timeout && cw.wait_for(0ms) == std::future_status::timeout);
  b0->close(); groups.remove("worldA/batch1", 0, b0);
  w0->close(); groups.remove("worldB/batch0", 0, w0);
  assert(!cb.get().received && !cw.get().received);
  assert(groups.take("worldA/batch1").empty() && groups.take("worldB/batch0").empty());
  auto closing = std::async(std::launch::async, [&] { return a1->call(query); });
  std::this_thread::sleep_for(2ms);
  a1->close(); groups.remove("worldA/batch0", 1, a1);
  assert(closing.wait_for(100ms) == std::future_status::ready && !closing.get().received);
  assert(groups.take("worldA/batch0").empty());
  a0->close(); groups.remove("worldA/batch0", 0, a0);
  std::cout << "provider primary drain, world+batch isolation, per-body correlation and unregister/close passed\n";
}
