// ros_sim_edge: the lightweight plant's ROS edge for one plant batch.
//
// ros_io serves one simulated robot role per plugin instance, so a hundred
// flight robots needed two hundred module threads, each woken at every plant
// output and every command poll. This plugin is the same edge for up to six
// robots in one instance: one thread, one non-blocking ROS pass, one read of
// the plant inputs. Per robot and role it runs the unchanged ros_io `RosIo`
// (see sim_edge_batch.hpp), so every topic, type, frame, noise stream, FCU
// service and provider generation behaves as before. ros_io.cpp is included
// as it is; only its plugin entry point is left out.
//
// Roles of one robot: `mavros` is the MAVROS-facing edge (map frame, FCU
// state, services, provider), `mocap` is the simulated mocap source (world
// frame, measurement noise) and, for a ground robot, also takes cmd_vel. Both
// roles of a robot read the same plant samples.
//
// Failure set: a failed step or module output write now faults the instance,
// that is the robots of one batch (at most six), where a per-robot edge
// faulted that robot's one or two edges. Those failures are configuration
// errors found at the first output, not run-time conditions.
#define XGC_ROS_IO_NO_ENTRY
#include "ros_io.cpp"

#include "sim_edge_batch.hpp"

namespace {

namespace sim_edge {

using xgc_sim_edge::PortSpec;
using xgc_sim_edge::RobotHost;
using xgc_sim_edge::Role;
using xgc_sim_edge::SplitConfig;
using xgc_sim_edge::Tee;

// A ros_io port that the batch carries, and where.
struct Carried {
  Port ros_io_port;
  uint32_t index;  // offset in the robot block, or the shared port index
  bool shared;
};

const Carried kCarried[] = {
    {kSimPose, xgc_sim_edge::kSimPose, false},
    {kSimVelocity, xgc_sim_edge::kSimVelocity, false},
    {kSimImu, xgc_sim_edge::kSimImu, false},
    {kSimFcuState, xgc_sim_edge::kSimFcuState, false},
    {kSimAttitudeTarget, xgc_sim_edge::kSimAttitudeTarget, false},
    {kAlgSetpoint, xgc_sim_edge::kAlgSetpoint, false},
    {kSimFcuRequest, xgc_sim_edge::kSimFcuRequest, false},
    {kAttitudeTargetFull, xgc_sim_edge::kAttitudeTargetFull, false},
    {kCmdVel, xgc_sim_edge::kCmdVel, false},
    {kSimFcuResult, xgc_sim_edge::kSimFcuResult, true},
    {kSimExtendedState, xgc_sim_edge::kSimExtendedState, true},
    {kSimProviderRequest, xgc_sim_edge::kSimProviderRequest, true},
    {kSimProviderResult, xgc_sim_edge::kSimProviderResult, true},
};

struct Member {
  uint32_t robot;
  Role role;
  std::unique_ptr<RobotHost> host;
  RosIo* io;
};

struct Edge {
  const xgc_host_api* host;
  std::vector<Member> members;
  Tee tee;
  std::vector<uint32_t> tee_ports;

  void log(xgc_log_level level, const std::string& message) const {
    host->log(host->host, level, ("ros_sim_edge: " + message).c_str());
  }

  void destroy_members() {
    for (auto& member : members) ::destroy(member.io);
    members.clear();
  }
  ~Edge() { destroy_members(); }

  xgc_status configure(const char* config) {
    const std::string text = config ? config : "";
    SplitConfig split;
    std::string error;
    if (!xgc_sim_edge::split_config(text, &split, &error)) {
      log(XGC_LOG_ERROR, error);
      return XGC_ERR;
    }
    for (uint32_t robot = 0; robot != xgc_sim_edge::kRobotsPerEdge; ++robot) {
      for (const Role role : {Role::kMavros, Role::kMocap}) {
        if (!split.present[robot][static_cast<size_t>(role)]) continue;
        auto shim = std::make_unique<RobotHost>();
        shim->bind(host, &tee, "ros_sim_edge[r" + std::to_string(robot) + " " + xgc_sim_edge::role_name(role) + "]: ", {});
        void* io = ::create(&shim->api);
        if (io == nullptr) {
          log(XGC_LOG_ERROR, "cannot allocate the edge of robot block " + std::to_string(robot));
          return XGC_ERR;
        }
        members.push_back({robot, role, std::move(shim), static_cast<RosIo*>(io)});
        Member& member = members.back();
        if (::configure(io, split.text_for(robot, role).c_str()) != XGC_OK) return XGC_ERR;
        // A port is carried exactly when the role configured its topic (the
        // same rule as the per-robot edge: no topic, no binding).
        std::vector<int32_t> map(static_cast<size_t>(::kPortCount), RobotHost::kUnmapped);
        for (const Carried& carried : kCarried) {
          if (!member.io->enabled(carried.ros_io_port)) continue;
          map[carried.ros_io_port] = static_cast<int32_t>(carried.shared ? carried.index
                                                                         : robot * xgc_sim_edge::kPortsPerRobot + carried.index);
        }
        member.host->map = std::move(map);
      }
    }
    if (members.empty()) {
      log(XGC_LOG_ERROR, "no robot role is configured");
      return XGC_ERR;
    }
    std::vector<bool> wanted(xgc_sim_edge::kPortCount, false);
    for (const auto& member : members)
      for (const int32_t port : member.host->map)
        if (port >= 0 && xgc_sim_edge::port_spec(static_cast<uint32_t>(port)).input) wanted[port] = true;
    tee_ports.clear();
    for (uint32_t port = 0; port != xgc_sim_edge::kPortCount; ++port)
      if (wanted[port]) tee_ports.push_back(port);
    return XGC_OK;
  }

  xgc_status activate() {
    for (auto& member : members)
      if (::activate(member.io) != XGC_OK) return XGC_ERR;
    return XGC_OK;
  }

  xgc_status step(const xgc_step_ctx* ctx) {
    // One read of the plant inputs; each role then reads its own cursor.
    tee.fill(host, tee_ports);
    xgc_status status = XGC_OK;
    for (auto& member : members) {
      member.host->reset_cursors();
      const xgc_status result = ::step(member.io, ctx);
      if (result != XGC_OK) status = result;
    }
    return status;
  }

  xgc_status deactivate() {
    for (auto& member : members) ::deactivate(member.io);
    return XGC_OK;
  }

  const char* domain_state() const {
    return members.empty() ? "down" : ::domain_state(members.front().io);
  }
};

void* create(const xgc_host_api* host) {
  try {
    auto* self = new Edge{};
    self->host = host;
    return self;
  } catch (...) {
    return nullptr;
  }
}

xgc_status configure(void* p, const char* config) {
  auto* self = static_cast<Edge*>(p);
  return guarded(self->host, "configure", [&] { return self->configure(config); });
}

xgc_status activate(void* p) {
  auto* self = static_cast<Edge*>(p);
  return guarded(self->host, "activate", [&] { return self->activate(); });
}

xgc_status step(void* p, const xgc_step_ctx* ctx) {
  auto* self = static_cast<Edge*>(p);
  return guarded(self->host, "step", [&] { return self->step(ctx); });
}

xgc_status deactivate(void* p) {
  auto* self = static_cast<Edge*>(p);
  return guarded(self->host, "deactivate", [&] { return self->deactivate(); });
}

void destroy(void* p) { delete static_cast<Edge*>(p); }

const char* domain_state(void* p) { return static_cast<const Edge*>(p)->domain_state(); }

struct Descriptor {
  std::array<std::string, xgc_sim_edge::kPortCount> names;
  std::array<xgc_port_decl, xgc_sim_edge::kPortCount> ports;
  xgc_plugin_vtbl vtbl;
  xgc_plugin_descriptor descriptor;

  Descriptor()
      : vtbl{sim_edge::create, sim_edge::configure, sim_edge::activate, sim_edge::step,
             sim_edge::deactivate, sim_edge::destroy, sim_edge::domain_state} {
    for (uint32_t index = 0; index != xgc_sim_edge::kPortCount; ++index) {
      names[index] = xgc_sim_edge::port_name(index);
      const PortSpec spec = xgc_sim_edge::port_spec(index);
      ports[index] = {names[index].c_str(), spec.input ? XGC_PORT_IN_OPTIONAL : XGC_PORT_OUT_OPTIONAL, spec.schema,
                      spec.qos};
    }
    descriptor = {XGC_RT_ABI_VERSION, xgc_sim_edge::kPortCount, "ros-sim-edge", "0.1.0", ports.data(), &vtbl};
  }
};

}  // namespace sim_edge

}  // namespace

extern "C" __attribute__((visibility("default"))) const xgc_plugin_descriptor* xgc_rt_plugin_v1(void) {
  static const sim_edge::Descriptor descriptor;
  return &descriptor.descriptor;
}
