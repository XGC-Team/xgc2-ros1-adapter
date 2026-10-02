// Real Noetic services and the actual native plant, through a boundary which
// stages all inputs before step(). Publications are only visible next step.
// The fixture also records any Host API call from a background ROS callback.
#include "xgc_rt.h"
#include "xgc_clock_source.h"
#include "xgc_schemas_v1.h"

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>

#include <array>
#include <atomic>
#include <cassert>
#include <chrono>
#include <cstring>
#include <deque>
#include <dlfcn.h>
#include <future>
#include <iostream>
#include <map>
#include <stdexcept>
#include <thread>
#include <vector>

using namespace std::chrono_literals;
using Bytes = std::vector<uint8_t>;
struct Packet { uint32_t port; Bytes bytes; };

struct Boundary {
  const std::thread::id owner{std::this_thread::get_id()};
  xgc_host_api api{};
  int64_t time{0};
  std::atomic<uint64_t> wrong_thread{0};
  std::array<std::deque<Bytes>, 64> incoming, staged;
  std::map<uint32_t, Bytes> state;
  std::vector<Packet> output;
  Bytes popped;

  Boundary() {
    api.abi_version = XGC_RT_ABI_VERSION;
    api.abi_minor = XGC_RT_ABI_MINOR;
    api.host = this;
    api.publish = [](void* p, uint32_t port, uint64_t, const uint8_t* bytes, uint32_t size) {
      auto& self = *static_cast<Boundary*>(p);
      if (std::this_thread::get_id() != self.owner) { ++self.wrong_thread; return XGC_ERR; }
      self.output.push_back({port, Bytes(bytes, bytes + size)});
      return XGC_OK;
    };
    api.next = [](void* p, uint32_t port, xgc_sample_view* out) {
      auto& self = *static_cast<Boundary*>(p);
      if (std::this_thread::get_id() != self.owner) { ++self.wrong_thread; return XGC_ERR; }
      if (self.staged.at(port).empty()) return XGC_ERR_AGAIN;
      self.popped = std::move(self.staged[port].front());
      self.staged[port].pop_front();
      *out = {};
      out->data = self.popped.data();
      out->len = self.popped.size();
      out->t_rx = self.time;
      return XGC_OK;
    };
    api.now = [](void* p) { return static_cast<Boundary*>(p)->time; };
    api.log = [](void*, xgc_log_level level, const char* text) {
      if (level >= XGC_LOG_ERROR) throw std::runtime_error(text);
    };
  }
  void stage(int64_t now) {
    time = now;
    for (size_t i = 0; i < staged.size(); ++i) staged[i].swap(incoming[i]);
    for (const auto& sample : state) staged[sample.first].push_back(sample.second);
  }
};

struct Library {
  void* handle;
  const xgc_plugin_descriptor* descriptor;
  explicit Library(const char* path) : handle(dlopen(path, RTLD_NOW | RTLD_LOCAL)) {
    if (!handle) throw std::runtime_error(dlerror());
    auto entry = reinterpret_cast<xgc_rt_plugin_v1_fn>(dlsym(handle, "xgc_rt_plugin_v1"));
    assert(entry);
    descriptor = entry();
  }
  ~Library() { dlclose(handle); }
  uint32_t port(const char* name) const {
    for (uint32_t i = 0; i < descriptor->port_count; ++i)
      if (std::strcmp(descriptor->ports[i].name, name) == 0) return i;
    throw std::runtime_error(std::string("missing port ") + name);
  }
};

template <class T> T decode(const Bytes& bytes) {
  assert(bytes.size() == sizeof(T));
  T value{};
  std::memcpy(&value, bytes.data(), sizeof value);
  return value;
}
template <class T> Bytes encode(const T& value) {
  const auto* bytes = reinterpret_cast<const uint8_t*>(&value);
  return {bytes, bytes + sizeof value};
}

struct Instance {
  Library& library;
  Boundary boundary;
  void* self;
  bool active{true};
  Instance(Library& lib, const std::string& config) : library(lib) {
    self = lib.descriptor->vtbl->create(&boundary.api);
    assert(self);
    assert(lib.descriptor->vtbl->configure(self, config.c_str()) == XGC_OK);
    assert(lib.descriptor->vtbl->activate(self) == XGC_OK);
  }
  void stop() {
    if (active) assert(library.descriptor->vtbl->deactivate(self) == XGC_OK);
    active = false;
  }
  ~Instance() { stop(); library.descriptor->vtbl->destroy(self); }
  void step(int64_t now) {
    if (!active) return;
    xgc_step_ctx ctx{};
    ctx.now = ctx.round_start = ctx.deadline = now;
    ctx.round = now / 1000000;
    ctx.round_advanced = 1;
    assert(library.descriptor->vtbl->step(self, &ctx) == XGC_OK);
  }
};

std::string edge_config(int index) {
  const auto ns = "/private_fcu/robot" + std::to_string(index) + "/mavros";
  return std::string("node_name = \"sim_fcu_service_test\"\nslice_ms = 0.001\n") +
         "sim_fcu_robot_index = " + std::to_string(index) + "\n" +
         "sim_fcu_timeout_ms = 250\nsim_fcu_freshness_ms = 100\n" +
         "sim_fcu_request_topic = \"" + ns + "\"\n" +
         "sim_fcu_state_topic = \"" + ns + "/state\"\n" +
         "sim_extended_state_topic = \"" + ns + "/extended_state\"\n";
}

int main(int argc, char** argv) {
  assert(argc == 3);
  ros::init(argc, argv, "sim_fcu_service_test", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  Library ros_io(argv[1]), plant_lib(argv[2]);
  assert(plant_lib.descriptor->port_count == 62);
  assert(std::strcmp(ros_io.descriptor->ports[ros_io.port("fcu_request")].schema_id, "xgc.fcu_request/1") == 0);
  assert(std::strcmp(ros_io.descriptor->ports[ros_io.port("sim_fcu_request")].schema_id, "xgc.fcu_request/2") == 0);
  const int64_t epoch = static_cast<int64_t>(ros::WallTime::now().toNSec());
  Instance plant(plant_lib, "model = \"fs150\"\nrobots = 6\nstep_ms = 1\noutput_ms = 1\nepoch_ns = " +
                           std::to_string(epoch) + "\n");
  Instance edge5(ros_io, edge_config(5)), edge4(ros_io, edge_config(4));
  std::array<mavros_msgs::State, 2> observed_state;
  std::array<mavros_msgs::ExtendedState, 2> observed_extended;
  auto state5 = nh.subscribe<mavros_msgs::State>("/private_fcu/robot5/mavros/state", 1,
      [&](const mavros_msgs::State::ConstPtr& m) { observed_state[0] = *m; });
  auto state4 = nh.subscribe<mavros_msgs::State>("/private_fcu/robot4/mavros/state", 1,
      [&](const mavros_msgs::State::ConstPtr& m) { observed_state[1] = *m; });
  auto extended5 = nh.subscribe<mavros_msgs::ExtendedState>("/private_fcu/robot5/mavros/extended_state", 1,
      [&](const mavros_msgs::ExtendedState::ConstPtr& m) { observed_extended[0] = *m; });
  auto extended4 = nh.subscribe<mavros_msgs::ExtendedState>("/private_fcu/robot4/mavros/extended_state", 1,
      [&](const mavros_msgs::ExtendedState::ConstPtr& m) { observed_extended[1] = *m; });
  auto bool_client = nh.serviceClient<mavros_msgs::CommandBool>("/private_fcu/robot5/mavros/cmd/arming");
  auto long_client = nh.serviceClient<mavros_msgs::CommandLong>("/private_fcu/robot5/mavros/cmd/command");
  auto mode_client = nh.serviceClient<mavros_msgs::SetMode>("/private_fcu/robot5/mavros/set_mode");
  assert(bool_client.waitForExistence(ros::Duration(3)));
  const auto start = std::chrono::steady_clock::now();
  uint64_t requests = 0, steps = 0;
  xgc_fcu_request_v2 last_request{};
  xgc_pose_v1 last_pose{};
  bool drop_results = false, replay_state = false, thrust = false;
  Bytes frozen_state;
  std::vector<Bytes> held_results;
  auto pump = [&] {
    const auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now() - start).count();
    const int64_t now = epoch + elapsed;
    if (thrust) {
      xgc_attitude_target_v2 target{};
      target.stamp = double(now) * 1e-9;
      target.q_wxyz[0] = 1;
      target.thrust = 0.8;
      target.type_mask = 7;
      plant.boundary.incoming[58].push_back(encode(target));
    }
    plant.boundary.stage(now); edge5.boundary.stage(now); edge4.boundary.stage(now);
    plant.step(now); edge5.step(now); edge4.step(now);
    for (const auto& packet : plant.boundary.output) {
      if (packet.port == 53) last_pose = decode<xgc_pose_v1>(packet.bytes);
      if (packet.port == 60) {
        if (drop_results) held_results.push_back(packet.bytes);
        else for (auto* edge : {&edge5, &edge4})
          edge->boundary.incoming[ros_io.port("sim_fcu_result")].push_back(packet.bytes);
      }
      if (packet.port == 61) for (auto* edge : {&edge5, &edge4})
        edge->boundary.state[ros_io.port("sim_extended_state")] = packet.bytes;
      if (packet.port == 56) {
        if (!replay_state) frozen_state = packet.bytes;
        edge5.boundary.state[ros_io.port("sim_fcu_state")] = frozen_state;
      }
      if (packet.port == 46) edge4.boundary.state[ros_io.port("sim_fcu_state")] = packet.bytes;
    }
    plant.boundary.output.clear();
    for (auto* edge : {&edge5, &edge4}) {
      for (const auto& packet : edge->boundary.output) {
        if (packet.port != ros_io.port("sim_fcu_request")) continue;
        last_request = decode<xgc_fcu_request_v2>(packet.bytes);
        assert(last_request.request_id & (uint64_t{1} << 63));
        ++requests;
        plant.boundary.incoming[edge == &edge5 ? 52 : 42].push_back(packet.bytes);
      }
      edge->boundary.output.clear();
    }
    ++steps;
    ros::spinOnce();
    std::this_thread::sleep_for(1ms);
  };
  auto wait = [&](const auto& predicate, std::chrono::milliseconds limit = 1000ms) {
    const auto end = std::chrono::steady_clock::now() + limit;
    while (!predicate()) {
      assert(std::chrono::steady_clock::now() < end);
      pump();
    }
  };
  auto invoke = [&](auto& client, auto& service) {
    auto call = std::async(std::launch::async, [&] { return client.call(service); });
    wait([&] { return call.wait_for(0ms) == std::future_status::ready; });
    assert(call.get());
  };
  wait([&] { return observed_state[0].connected && observed_state[1].connected &&
                    observed_extended[0].landed_state == 1 && observed_extended[1].landed_state == 1; });
  assert(observed_state[0].system_status == 3 && observed_state[1].system_status == 3);

  // Hold the executed result: publication and even an observed armed state
  // must not complete the service. An otherwise matching native low-half ID
  // must not complete it either.
  drop_results = true;
  mavros_msgs::CommandBool arm;
  arm.request.value = true;
  auto first = std::async(std::launch::async, [&] { return bool_client.call(arm); });
  wait([&] { return !held_results.empty(); });
  auto low = decode<xgc_fcu_result_v1>(held_results.back());
  low.request_id &= ~(uint64_t{1} << 63);
  edge5.boundary.incoming[ros_io.port("sim_fcu_result")].push_back(encode(low));
  const auto after = std::chrono::steady_clock::now() + 20ms;
  while (std::chrono::steady_clock::now() < after) pump();
  assert(first.wait_for(0ms) == std::future_status::timeout);
  drop_results = false;
  for (const auto& bytes : held_results) edge5.boundary.incoming[ros_io.port("sim_fcu_result")].push_back(bytes);
  held_results.clear();
  wait([&] { return first.wait_for(0ms) == std::future_status::ready; });
  assert(first.get() && arm.response.success && arm.response.result == 0);
  wait([&] { return observed_state[0].armed; });
  assert(observed_state[0].system_status == 4 && !observed_state[1].armed);

  mavros_msgs::CommandLong command;
  command.request.command = 400;
  command.request.param1 = 0;
  invoke(long_client, command);
  assert(command.response.success && command.response.result == 0);
  wait([&] { return !observed_state[0].armed; });
  command.request.param1 = 1;
  invoke(long_client, command);
  assert(command.response.success && command.response.result == 0);
  wait([&] { return observed_state[0].armed; });
  arm.request.value = false;
  invoke(bool_client, arm);
  assert(arm.response.success && arm.response.result == 0);
  wait([&] { return !observed_state[0].armed; });
  arm.request.value = true;
  invoke(bool_client, arm);
  assert(arm.response.success);
  wait([&] { return observed_state[0].armed; });
  command.request.param1 = 0;
  command.request.param2 = 21196;
  invoke(long_client, command);
  assert(!command.response.success && command.response.result == 3 && last_request.flags == 1);
  assert(observed_state[0].armed);  // Force was refused, never downgraded to disarm.
  const auto before_invalid = requests;
  command.request.param2 = 21195;
  invoke(long_client, command);
  assert(!command.response.success && command.response.result == 3 && requests == before_invalid);
  command.request.param2 = 0;

  mavros_msgs::SetMode mode;
  mode.request.custom_mode = "NOT_MODELLED";
  const auto old_mode = observed_state[0].mode;
  invoke(mode_client, mode);
  assert(mode.response.mode_sent);
  for (int i = 0; i < 20; ++i) pump();
  assert(observed_state[0].mode == old_mode);
  thrust = true;
  for (int i = 0; i < 10; ++i) pump();
  mode.request.custom_mode = "OFFBOARD";
  invoke(mode_client, mode);
  assert(mode.response.mode_sent);
  wait([&] { return observed_state[0].mode == "OFFBOARD" && last_pose.position[2] > 0.05; }, 1500ms);
  arm.request.value = false;
  invoke(bool_client, arm);
  assert(!arm.response.success && arm.response.result == 2);
  invoke(long_client, command);
  assert(!command.response.success && command.response.result == 2);
  wait([&] { return observed_extended[0].landed_state == 2; });
  assert(observed_extended[0].vtol_state == 0 && observed_extended[1].landed_state == 1);
  mode.request.custom_mode = "AUTO.LAND";
  invoke(mode_client, mode);
  wait([&] { return observed_extended[0].landed_state == 4; });

  drop_results = true;
  const auto timeout_started = std::chrono::steady_clock::now();
  invoke(long_client, command);
  assert(!command.response.success && command.response.result == 4);
  assert(std::chrono::steady_clock::now() - timeout_started < 800ms);
  drop_results = false;
  held_results.clear();
  replay_state = true;
  const auto age = std::chrono::steady_clock::now() + 140ms;
  while (std::chrono::steady_clock::now() < age) pump();
  const auto before_stale = requests;
  invoke(bool_client, arm);
  assert(!arm.response.success && arm.response.result == 4 && requests == before_stale);
  replay_state = false;
  for (int i = 0; i < 20; ++i) pump();

  auto clock_entry = reinterpret_cast<xgc_clock_source_entry_v1>(dlsym(ros_io.handle, XGC_CLOCK_SOURCE_ENTRY));
  assert(clock_entry);
  auto clock = clock_entry();
  auto source = clock->vtbl->create();
  ros::param::set("/use_sim_time", true);  // Private master; start the actual output-gate owner.
  xgc_clock_observation_v1 observation{};
  const auto clock_started = clock->vtbl->start(source,
      "node_name = \"sim_fcu_service_test\"\ntopic = \"/private_fcu/clock\"\nexpected_publisher = \"/sim_fcu_service_test\"\nqueue_capacity = 64\n", &observation);
  if (clock_started != XGC_CLOCK_OK) throw std::runtime_error(observation.error);
  const auto before_closed = requests;
  invoke(bool_client, arm);
  assert(!arm.response.success && arm.response.result == 4 && requests == before_closed);
  assert(clock->vtbl->set_gate(source, XGC_CLOCK_GATE_OPEN) == XGC_CLOCK_OK);
  for (int i = 0; i < 20; ++i) pump();

  drop_results = true;
  const auto before_stop = requests;
  auto pending = std::async(std::launch::async, [&] { return bool_client.call(arm); });
  wait([&] { return requests > before_stop; });
  const auto stop_started = std::chrono::steady_clock::now();
  edge5.stop();
  assert(pending.wait_for(200ms) == std::future_status::ready);
  const bool stop_transport = pending.get();
  assert(!stop_transport || (!arm.response.success && arm.response.result == 4));
  assert(std::chrono::steady_clock::now() - stop_started < 300ms);
  clock->vtbl->stop(source);
  clock->vtbl->destroy(source);
  assert(edge5.boundary.wrong_thread == 0 && edge4.boundary.wrong_thread == 0);
  std::cout << "{\"result\":\"passed\",\"steps\":" << steps
            << ",\"requests\":" << requests << ",\"model_robots\":6,\"plant_ports\":62,"
            << "\"host_background_calls\":0,\"actual_airborne_z\":" << last_pose.position[2]
            << ",\"force_result\":3,\"airborne_disarm_result\":2,\"timeout_result\":4,"
            << "\"stop_transport_response\":" << (stop_transport ? "true" : "false") << "}\n";
}
