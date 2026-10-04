// ROS-free parts of the batched simulation edge (ros_sim_edge.cpp).
//
// One edge instance serves up to six simulated robots of one plant batch. It
// holds one ros_io `RosIo` per robot and role (the MAVROS-facing edge and the
// mocap-source edge of a flight robot; the one edge of a ground robot), so a
// robot publishes exactly the topics, types and values its own per-robot
// edges published. What is shared is the thread, the ROS callback pass and
// one read of every plant input.
//
// Port layout (stride 9 per robot block, shared ports last), 58 of 64:
//   block r:  sim_pose, sim_velocity, sim_imu, sim_fcu_state,
//             sim_attitude_target            (inputs from the plant)
//             alg_setpoint, sim_fcu_request, attitude_target_full, cmd_vel
//                                              (outputs to the plant)
//   shared:   sim_fcu_result, sim_extended_state, sim_provider_request,
//             sim_provider_result
// Block 0 keeps the plain port names; block r > 0 appends `_r` (the
// lightweight-vehicle convention).
//
// Config: keys `r<k>_<role>_<key>` (k = robot block, role = mavros or mocap)
// belong to one `RosIo`; every other key is global and reaches all of them.
#pragma once

#include <array>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "xgc_rt.h"

namespace xgc_sim_edge {

inline constexpr uint32_t kRobotsPerEdge = 6;
inline constexpr uint32_t kPortsPerRobot = 9;
inline constexpr uint32_t kSharedBase = kRobotsPerEdge * kPortsPerRobot;  // 54
inline constexpr uint32_t kPortCount = kSharedBase + 4;                   // 58
static_assert(kPortCount <= XGC_RT_MAX_PORTS, "the edge must fit the plugin ABI");

enum RobotPort : uint32_t {
  kSimPose = 0,
  kSimVelocity,
  kSimImu,
  kSimFcuState,
  kSimAttitudeTarget,
  kAlgSetpoint,
  kSimFcuRequest,
  kAttitudeTargetFull,
  kCmdVel,
};

enum SharedPort : uint32_t {
  kSimFcuResult = kSharedBase,
  kSimExtendedState,
  kSimProviderRequest,
  kSimProviderResult,
};

inline const char* robot_port_name(uint32_t port) {
  static const char* const names[kPortsPerRobot] = {
      "sim_pose",     "sim_velocity",         "sim_imu",              "sim_fcu_state", "sim_attitude_target",
      "alg_setpoint", "sim_fcu_request",      "attitude_target_full", "cmd_vel"};
  return names[port];
}

inline const char* shared_port_name(uint32_t port) {
  static const char* const names[4] = {"sim_fcu_result", "sim_extended_state", "sim_provider_request",
                                       "sim_provider_result"};
  return names[port - kSharedBase];
}

// The manifest name of an ABI port index.
inline std::string port_name(uint32_t index) {
  if (index >= kSharedBase) return shared_port_name(index);
  const uint32_t block = index / kPortsPerRobot;
  std::string name = robot_port_name(index % kPortsPerRobot);
  if (block != 0) name += "_" + std::to_string(block);
  return name;
}

// Direction, schema and QoS of each port, as declared by the single-robot
// ros_io edge. Ports 0..4 of a block and the shared results are inputs.
struct PortSpec {
  bool input;
  const char* schema;
  xgc_qos qos;
};

inline PortSpec robot_port_spec(uint32_t port) {
  static const PortSpec specs[kPortsPerRobot] = {
      {true, "xgc.pose/1", XGC_QOS_STATE},
      {true, "xgc.twist/1", XGC_QOS_STATE},
      {true, "xgc.imu/1", XGC_QOS_STATE},
      {true, "xgc.fcu_state/1", XGC_QOS_STATE},
      {true, "xgc.attitude_target/2", XGC_QOS_STATE},
      {false, "xgc.position_target/1", XGC_QOS_CONTROL},
      {false, "xgc.fcu_request/2", XGC_QOS_EVENT},
      {false, "xgc.attitude_target/2", XGC_QOS_CONTROL},
      {false, "xgc.twist/1", XGC_QOS_CONTROL},
  };
  return specs[port];
}

inline PortSpec shared_port_spec(uint32_t port) {
  static const PortSpec specs[4] = {
      {true, "xgc.fcu_result/1", XGC_QOS_EVENT},
      {true, "xgc.fcu_extended_state/1", XGC_QOS_STATE},
      {false, "xgc.sim_provider_request/1", XGC_QOS_EVENT},
      {true, "xgc.sim_provider_result/1", XGC_QOS_EVENT},
  };
  return specs[port - kSharedBase];
}

inline PortSpec port_spec(uint32_t index) {
  return index >= kSharedBase ? shared_port_spec(index) : robot_port_spec(index % kPortsPerRobot);
}

// The role of one `RosIo` inside a batch.
enum class Role { kMavros, kMocap };

inline const char* role_name(Role role) { return role == Role::kMavros ? "mavros" : "mocap"; }

// `r<k>_<role>_<key>` -> (k, role, key). False for a global key.
inline bool robot_key(const std::string& key, uint32_t* robot, Role* role, std::string* rest) {
  if (key.size() < 6 || key[0] != 'r' || key[1] < '0' || key[1] > '9' || key[2] != '_') return false;
  const uint32_t block = key[1] - '0';
  const std::string tail = key.substr(3);
  Role found;
  size_t skip;
  if (tail.compare(0, 6, "mavros") == 0) {
    found = Role::kMavros;
    skip = 6;
  } else if (tail.compare(0, 5, "mocap") == 0) {
    found = Role::kMocap;
    skip = 5;
  } else {
    return false;
  }
  if (tail.size() <= skip + 1 || tail[skip] != '_') return false;
  *robot = block;
  *role = found;
  *rest = tail.substr(skip + 1);
  return true;
}

// Splits the plugin config (flat `key = value` lines) into the global lines
// and the lines of each robot role. Values are kept verbatim.
struct SplitConfig {
  std::string global;
  // role_lines[robot][role] is empty when the role has no key.
  std::array<std::array<std::string, 2>, kRobotsPerEdge> role_lines;
  std::array<std::array<bool, 2>, kRobotsPerEdge> present{};

  // The config text one RosIo is configured with: the global lines first,
  // then its own with the prefix removed.
  std::string text_for(uint32_t robot, Role role) const {
    return global + role_lines[robot][static_cast<size_t>(role)];
  }
};

// Returns false (with `error`) for an out-of-range robot block or a line
// that is not `key = value`.
inline bool split_config(const std::string& text, SplitConfig* out, std::string* error) {
  size_t begin = 0;
  while (begin < text.size()) {
    size_t end = text.find('\n', begin);
    if (end == std::string::npos) end = text.size();
    const std::string line = text.substr(begin, end - begin);
    begin = end + 1;
    if (line.empty()) continue;
    const size_t eq = line.find('=');
    if (eq == std::string::npos) {
      *error = "config line without '=': " + line;
      return false;
    }
    size_t key_end = eq;
    while (key_end > 0 && line[key_end - 1] == ' ') --key_end;
    const std::string key = line.substr(0, key_end);
    uint32_t robot = 0;
    Role role = Role::kMavros;
    std::string rest;
    if (!robot_key(key, &robot, &role, &rest)) {
      // `r<digit>_...` is the robot namespace: a misspelled role must not
      // silently become a global key.
      if (key.size() > 2 && key[0] == 'r' && key[1] >= '0' && key[1] <= '9' && key[2] == '_') {
        *error = "config key " + key + " names no robot role (mavros or mocap)";
        return false;
      }
      out->global += line + "\n";
      continue;
    }
    if (robot >= kRobotsPerEdge) {
      *error = "robot block " + std::to_string(robot) + " is outside 0.." + std::to_string(kRobotsPerEdge - 1);
      return false;
    }
    out->role_lines[robot][static_cast<size_t>(role)] += rest + line.substr(key_end) + "\n";
    out->present[robot][static_cast<size_t>(role)] = true;
  }
  return true;
}

// One read of every plant input per step. Samples are copied into a flat
// arena per port (no per-sample allocation after the first steps), then each
// RosIo reads its own cursor, so two roles of one robot, or six robots on a
// shared result port, each see every sample as their own queue would have.
class Tee {
 public:
  void fill(const xgc_host_api* host, const std::vector<uint32_t>& ports) {
    for (const uint32_t port : ports) {
      Port& slot = ports_[port];
      slot.items.clear();
      slot.bytes.clear();
      xgc_sample_view view;
      while (host->next(host->host, port, &view) == XGC_OK) {
        Item item{view, static_cast<uint32_t>(slot.bytes.size())};
        if (view.len != 0 && view.data != nullptr) slot.bytes.insert(slot.bytes.end(), view.data, view.data + view.len);
        else item.view.len = 0;
        slot.items.push_back(item);
      }
    }
  }

  size_t size(uint32_t port) const { return ports_[port].items.size(); }

  // The index-th sample of `port` of this step, with a data pointer into the
  // arena that stays valid until the next fill().
  xgc_sample_view view(uint32_t port, size_t index) const {
    const Port& slot = ports_[port];
    xgc_sample_view view = slot.items[index].view;
    view.data = slot.bytes.data() + slot.items[index].offset;
    return view;
  }

 private:
  struct Item {
    xgc_sample_view view;
    uint32_t offset;
  };
  struct Port {
    std::vector<Item> items;
    std::vector<uint8_t> bytes;
  };
  std::array<Port, kPortCount> ports_;
};

// The host API one RosIo sees. Its ports are the single-robot ros_io port
// numbers; `map` sends each to an ABI port of the batch (-1: not bound to
// this robot role). Inputs are served from the tee, outputs and logs go to
// the real host.
struct RobotHost {
  static constexpr int32_t kUnmapped = -1;

  const xgc_host_api* real{nullptr};
  Tee* tee{nullptr};
  std::string log_prefix;
  std::vector<int32_t> map;           // by ros_io port number
  std::array<uint32_t, kPortCount> cursor{};
  xgc_host_api api{};

  void reset_cursors() { cursor.fill(0); }

  static xgc_status next(void* self, uint32_t port, xgc_sample_view* out) {
    auto* shim = static_cast<RobotHost*>(self);
    if (out == nullptr) return XGC_ERR_INVALID;
    const int32_t real_port = shim->mapped(port);
    if (real_port < 0 || !port_spec(static_cast<uint32_t>(real_port)).input) return XGC_ERR_AGAIN;
    uint32_t& at = shim->cursor[real_port];
    if (at >= shim->tee->size(real_port)) return XGC_ERR_AGAIN;
    *out = shim->tee->view(real_port, at++);
    return XGC_OK;
  }

  static xgc_status publish(void* self, uint32_t port, uint64_t round, const uint8_t* data, uint32_t len) {
    auto* shim = static_cast<RobotHost*>(self);
    const int32_t real_port = shim->mapped(port);
    if (real_port < 0 || port_spec(static_cast<uint32_t>(real_port)).input) return XGC_ERR_INVALID;
    return shim->real->publish(shim->real->host, static_cast<uint32_t>(real_port), round, data, len);
  }

  static int64_t now(void* self) {
    auto* shim = static_cast<RobotHost*>(self);
    return shim->real->now(shim->real->host);
  }

  static void log(void* self, xgc_log_level level, const char* message) {
    auto* shim = static_cast<RobotHost*>(self);
    shim->real->log(shim->real->host, level, (shim->log_prefix + message).c_str());
  }

  static void request_degrade(void* self, const char* reason) {
    auto* shim = static_cast<RobotHost*>(self);
    shim->real->request_degrade(shim->real->host, reason);
  }

  static void request_recover(void* self) {
    auto* shim = static_cast<RobotHost*>(self);
    shim->real->request_recover(shim->real->host);
  }

  static uint32_t port_origins(void*, uint32_t, uint16_t*, uint32_t) { return 0; }

  static uint16_t node_id(void* self) {
    auto* shim = static_cast<RobotHost*>(self);
    return shim->real->node_id(shim->real->host);
  }

  void bind(const xgc_host_api* host, Tee* shared_tee, std::string prefix, std::vector<int32_t> ros_io_map) {
    real = host;
    tee = shared_tee;
    log_prefix = std::move(prefix);
    map = std::move(ros_io_map);
    api = *host;
    api.host = this;
    api.publish = &RobotHost::publish;
    api.next = &RobotHost::next;
    api.now = &RobotHost::now;
    api.log = &RobotHost::log;
    api.request_degrade = &RobotHost::request_degrade;
    api.request_recover = &RobotHost::request_recover;
    api.port_origins = &RobotHost::port_origins;
    api.node_id = &RobotHost::node_id;
  }

 private:
  int32_t mapped(uint32_t port) const { return port < map.size() ? map[port] : kUnmapped; }
};

}  // namespace xgc_sim_edge
