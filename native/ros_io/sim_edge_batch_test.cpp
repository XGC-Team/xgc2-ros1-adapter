// ROS-free checks of the batched simulation edge: port layout, config split,
// the one-read plant input tee and the per-robot host each ros_io edge sees.
#undef NDEBUG
#include <cassert>
#include <cstdio>
#include <cstring>
#include <set>
#include <string>
#include <vector>

#include "sim_edge_batch.hpp"

using namespace xgc_sim_edge;

namespace {

// A host with scripted input queues and a record of what was published.
struct FakeHost {
  struct Sample {
    std::vector<uint8_t> bytes;
    int64_t t_rx;
  };
  std::vector<std::vector<Sample>> queues = std::vector<std::vector<Sample>>(kPortCount);
  std::vector<size_t> cursor = std::vector<size_t>(kPortCount, 0);
  struct Published {
    uint32_t port;
    uint64_t round;
    std::vector<uint8_t> bytes;
  };
  std::vector<Published> published;
  std::vector<std::string> logs;
  uint32_t stale_reads = 0;
  xgc_host_api api{};

  FakeHost() {
    api.abi_version = XGC_RT_ABI_VERSION;
    api.abi_minor = XGC_RT_ABI_MINOR;
    api.host = this;
    api.publish = [](void* self, uint32_t port, uint64_t round, const uint8_t* data, uint32_t len) {
      static_cast<FakeHost*>(self)->published.push_back({port, round, std::vector<uint8_t>(data, data + len)});
      return XGC_OK;
    };
    api.next = [](void* self, uint32_t port, xgc_sample_view* out) {
      auto* host = static_cast<FakeHost*>(self);
      if (port >= kPortCount || host->cursor[port] >= host->queues[port].size()) return XGC_ERR_AGAIN;
      const Sample& sample = host->queues[port][host->cursor[port]++];
      *out = {};
      out->len = static_cast<uint32_t>(sample.bytes.size());
      out->t_rx = sample.t_rx;
      out->data = sample.bytes.data();
      return XGC_OK;
    };
    api.now = [](void*) { return int64_t{42}; };
    api.log = [](void* self, xgc_log_level, const char* message) {
      static_cast<FakeHost*>(self)->logs.push_back(message);
    };
    api.request_degrade = [](void*, const char*) {};
    api.request_recover = [](void*) {};
    api.port_origins = [](void*, uint32_t, uint16_t*, uint32_t) { return uint32_t{0}; };
    api.node_id = [](void*) { return uint16_t{0}; };
  }
  void push(uint32_t port, std::vector<uint8_t> bytes, int64_t t_rx = 0) { queues[port].push_back({std::move(bytes), t_rx}); }
  void new_step() { std::fill(cursor.begin(), cursor.end(), 0); for (auto& q : queues) q.clear(); }
};

void layout() {
  static_assert(kPortCount == 58, "6 blocks of 9 plus 4 shared");
  std::set<std::string> names;
  for (uint32_t i = 0; i != kPortCount; ++i) {
    const std::string name = port_name(i);
    assert(names.insert(name).second);
  }
  assert(port_name(0) == "sim_pose" && port_name(1) == "sim_velocity" && port_name(8) == "cmd_vel");
  assert(port_name(9) == "sim_pose_1" && port_name(9 * 5 + 7) == "attitude_target_full_5");
  assert(port_name(kSimFcuResult) == "sim_fcu_result" && port_name(kSimProviderResult) == "sim_provider_result");
  // Inputs are the plant's state; outputs are the ROS commands and requests.
  for (uint32_t block = 0; block != kRobotsPerEdge; ++block) {
    for (uint32_t port = 0; port != kPortsPerRobot; ++port)
      assert(port_spec(block * kPortsPerRobot + port).input == (port <= kSimAttitudeTarget));
  }
  assert(port_spec(kSimFcuResult).input && port_spec(kSimExtendedState).input);
  assert(!port_spec(kSimProviderRequest).input && port_spec(kSimProviderResult).input);
  assert(std::strcmp(port_spec(kSimFcuRequest).schema, "xgc.fcu_request/2") == 0);
  assert(port_spec(kSimFcuRequest).qos == XGC_QOS_EVENT && port_spec(kSimPose).qos == XGC_QOS_STATE);
  assert(port_spec(9 * 3 + kCmdVel).qos == XGC_QOS_CONTROL);
}

void config_split() {
  const std::string text =
      "node_name = \"xgc_lightweight_plant\"\n"
      "slice_ms = 0.001\n"
      "r0_mavros_sim_pose_topic = \"/uav1/mavros/local_position/pose\"\n"
      "r0_mavros_frame_id = \"map\"\n"
      "r0_mocap_sim_pose_topic = \"/vrpn_client_node/uav1/pose\"\n"
      "r0_mocap_frame_id = \"world\"\n"
      "r2_mocap_sim_mocap_position_stddev_m = [1e-07, 1e-07, 1e-07]\n"
      "r2_mocap_sim_mocap_noise_seed = 7\n";
  SplitConfig split;
  std::string error;
  assert(split_config(text, &split, &error));
  assert(split.global == "node_name = \"xgc_lightweight_plant\"\nslice_ms = 0.001\n");
  assert(split.present[0][0] && split.present[0][1] && !split.present[1][0] && !split.present[2][0] && split.present[2][1]);
  assert(split.text_for(0, Role::kMavros) ==
         split.global + "sim_pose_topic = \"/uav1/mavros/local_position/pose\"\nframe_id = \"map\"\n");
  assert(split.text_for(0, Role::kMocap) ==
         split.global + "sim_pose_topic = \"/vrpn_client_node/uav1/pose\"\nframe_id = \"world\"\n");
  assert(split.text_for(2, Role::kMocap) ==
         split.global + "sim_mocap_position_stddev_m = [1e-07, 1e-07, 1e-07]\nsim_mocap_noise_seed = 7\n");
  // A key of one robot never reaches another robot or the other role.
  assert(split.text_for(1, Role::kMavros) == split.global);
  assert(split.text_for(0, Role::kMavros).find("vrpn") == std::string::npos);

  for (const char* bad : {"r6_mavros_sim_pose_topic = \"/x\"\n", "r1_other_sim_pose_topic = \"/x\"\n",
                          "r1_mocap_ = 1\n", "not a key value\n"}) {
    SplitConfig rejected;
    assert(!split_config(bad, &rejected, &error) && !error.empty());
  }
  // `robot_key` is exact: a global key that merely contains the text stays global.
  SplitConfig global_only;
  assert(split_config("sim_extended_state_topic = \"/a\"\nsim_body_pose_topic = \"/b\"\n", &global_only, &error));
  assert(global_only.global.find("sim_body_pose_topic") != std::string::npos);
}

std::vector<uint8_t> bytes(std::initializer_list<int> values) {
  std::vector<uint8_t> out;
  for (int v : values) out.push_back(static_cast<uint8_t>(v));
  return out;
}

void tee_and_robot_hosts() {
  FakeHost host;
  Tee tee;
  // Robot 1's mavros and mocap roles both read its pose; robot 2 has its own.
  const uint32_t pose1 = 1 * kPortsPerRobot + kSimPose, pose2 = 2 * kPortsPerRobot + kSimPose;
  const std::vector<uint32_t> inputs = {pose1, pose2, kSimFcuResult};

  RobotHost mavros, mocap, other;
  // ros_io port numbers used here: 0 = pose-like input, 1 = request output.
  mavros.bind(&host.api, &tee, "m: ", {static_cast<int32_t>(pose1), static_cast<int32_t>(1 * kPortsPerRobot + kAlgSetpoint),
                                       static_cast<int32_t>(kSimFcuResult)});
  mocap.bind(&host.api, &tee, "c: ", {static_cast<int32_t>(pose1), RobotHost::kUnmapped});
  other.bind(&host.api, &tee, "o: ", {static_cast<int32_t>(pose2), RobotHost::kUnmapped, static_cast<int32_t>(kSimFcuResult)});

  host.push(pose1, bytes({1, 2, 3}), 10);
  host.push(pose1, bytes({4, 5, 6}), 20);
  host.push(pose2, bytes({9}), 30);
  host.push(kSimFcuResult, bytes({7, 7}), 40);
  tee.fill(&host.api, inputs);
  // The host's own queues are drained by the single read.
  xgc_sample_view probe;
  assert(host.api.next(host.api.host, pose1, &probe) == XGC_ERR_AGAIN);

  for (RobotHost* shim : {&mavros, &mocap, &other}) shim->reset_cursors();
  xgc_sample_view view;
  // Both roles of robot 1 see both samples, in order, with their stamps.
  for (RobotHost* role : {&mavros, &mocap}) {
    assert(role->api.next(role->api.host, 0, &view) == XGC_OK && view.len == 3 && view.data[0] == 1 && view.t_rx == 10);
    assert(role->api.next(role->api.host, 0, &view) == XGC_OK && view.len == 3 && view.data[0] == 4 && view.t_rx == 20);
    assert(role->api.next(role->api.host, 0, &view) == XGC_ERR_AGAIN);
  }
  // Robot 2 never sees robot 1's samples; the shared result reaches both
  // roles that carry it and is absent for the one that does not.
  assert(other.api.next(other.api.host, 0, &view) == XGC_OK && view.len == 1 && view.data[0] == 9);
  assert(other.api.next(other.api.host, 0, &view) == XGC_ERR_AGAIN);
  assert(mavros.api.next(mavros.api.host, 2, &view) == XGC_OK && view.len == 2 && view.data[1] == 7);
  assert(other.api.next(other.api.host, 2, &view) == XGC_OK && view.len == 2);
  assert(mocap.api.next(mocap.api.host, 2, &view) == XGC_ERR_AGAIN);  // port beyond its map
  assert(mocap.api.next(mocap.api.host, 1, &view) == XGC_ERR_AGAIN);  // unmapped
  assert(mocap.api.next(mocap.api.host, 0, nullptr) == XGC_ERR_INVALID);

  // Outputs reach the mapped ABI port of this robot only; an input port, an
  // unmapped port and an out-of-range port are rejected.
  const uint8_t payload[2] = {0xAB, 0xCD};
  assert(mavros.api.publish(mavros.api.host, 1, 5, payload, 2) == XGC_OK);
  assert(host.published.size() == 1 && host.published[0].port == 1 * kPortsPerRobot + kAlgSetpoint &&
         host.published[0].round == 5 && host.published[0].bytes == std::vector<uint8_t>({0xAB, 0xCD}));
  assert(mavros.api.publish(mavros.api.host, 0, 5, payload, 2) == XGC_ERR_INVALID);   // input port
  assert(mocap.api.publish(mocap.api.host, 1, 5, payload, 2) == XGC_ERR_INVALID);     // unmapped
  assert(mocap.api.publish(mocap.api.host, 99, 5, payload, 2) == XGC_ERR_INVALID);    // out of range
  assert(host.published.size() == 1);

  // Time and logs go to the real host; the log names its robot and role.
  assert(mavros.api.now(mavros.api.host) == 42);
  mavros.api.log(mavros.api.host, XGC_LOG_INFO, "hello");
  assert(host.logs.size() == 1 && host.logs[0] == "m: hello");

  // The next step starts from fresh buffers and cursors; a held view of the
  // previous step is not used again.
  host.new_step();
  host.push(pose1, bytes({8}), 50);
  tee.fill(&host.api, inputs);
  mavros.reset_cursors();
  assert(mavros.api.next(mavros.api.host, 0, &view) == XGC_OK && view.len == 1 && view.data[0] == 8 && view.t_rx == 50);
  assert(mavros.api.next(mavros.api.host, 0, &view) == XGC_ERR_AGAIN);
  assert(tee.size(pose2) == 0 && tee.size(kSimFcuResult) == 0);
}

void empty_payloads_survive() {
  FakeHost host;
  Tee tee;
  host.push(kSimFcuResult, {}, 3);
  tee.fill(&host.api, {kSimFcuResult});
  RobotHost shim;
  shim.bind(&host.api, &tee, "", {static_cast<int32_t>(kSimFcuResult)});
  shim.reset_cursors();
  xgc_sample_view view;
  assert(shim.api.next(shim.api.host, 0, &view) == XGC_OK && view.len == 0 && view.t_rx == 3);
}

}  // namespace

int main() {
  layout();
  config_split();
  tee_and_robot_hosts();
  empty_payloads_survive();
  std::puts("sim_edge_batch_test passed");
  return 0;
}
