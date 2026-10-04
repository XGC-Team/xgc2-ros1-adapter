// ros_io: the aggregator's ROS edge. It does ordinary ROS subscribe and
// publish and copies data between topics and module I/O. It is not the
// ros1_bridge package and there is no separate bridge process: domain
// generic sensor, FCU and provider edges are owned here. Domain ROS plugins
// are assembled by their own algorithm workspace.
//
// Every port is optional and is enabled by `<port>_topic` in the config (an
// enabled port must also be bound in the manifest, and a bound port needs its
// topic):
//
//   ROS -> module outputs
//     imu              sensor_msgs/Imu                      -> xgc.imu/1
//     pose             geometry_msgs/PoseStamped            -> xgc.pose/1
//     attitude_target  mavros_msgs/AttitudeTarget           -> xgc.attitude_target/1
//     fcu_state        mavros_msgs/State                    -> xgc.fcu_state/1
//     local_pose       geometry_msgs/PoseStamped            -> xgc.pose/1
//     local_velocity   geometry_msgs/TwistStamped           -> xgc.twist/1
//     fcu_imu          sensor_msgs/Imu                      -> xgc.imu/1
//     battery          sensor_msgs/BatteryState             -> xgc.battery/1
//     command          std_msgs/String                      -> xgc.command/1
//     alg_setpoint     mavros_msgs/PositionTarget           -> xgc.position_target/1 (a planner's setpoint)
//     ref_analytic     multirotor_reference_trajectory_msgs/AnalyticReference       -> xgc.ref.analytic/1
//     ref_sampled      multirotor_reference_trajectory_msgs/SampledReference        -> xgc.ref.sampled/1
//     ref_reset        std_msgs/Empty                        -> xgc.ref.reset/1
//     hover_thrust     hover_thrust_estimator_msgs/HoverThrustEstimate -> xgc.hover_thrust/1
//     controller_state std_msgs/String (custom/statustext) -> xgc.controller_status/1 (stamp = receipt)
//     cmd_vel          geometry_msgs/Twist                   -> xgc.twist/1 (stamp = receipt)
//   module inputs -> ROS
//     vision_pose      xgc.pose/1                   -> geometry_msgs/PoseStamped (frame `frame_id`)
//     rigid_state_estimate xgc.rigid_state_estimate/1 -> rigid_state_estimator_msgs/RigidStateEstimate
//     setpoint         xgc.position_target/1        -> mavros_msgs/PositionTarget (frame "map")
//     attitude_rate    xgc.body_rate_thrust/1       -> mavros_msgs/AttitudeTarget (attitude ignored)
//     status           xgc.controller_status/1      -> std_msgs/String
//     fcu_request      xgc.fcu_request/1            -> MAVROS service calls. Here the "topic" is
//                      the MAVROS namespace (e.g. /uav1/mavros): arm/disarm calls
//                      <ns>/cmd/command (MAV_CMD 400), set_mode calls <ns>/set_mode.
//                      Calls run in order on a worker thread, as the ROS node ran them
//                      off its control loop; nothing is reported back to the module.
//     ref_status       xgc.ref.status/1             -> multirotor_reference_trajectory_msgs/ReferenceStatus (latched)
//     ref_active_analytic   xgc.ref.analytic/1      -> .../AnalyticReference (latched)
//     ref_active_sampled    xgc.ref.sampled/1       -> .../SampledReference (latched)
//   (the ref_* outputs are latched, as the reference trajectory node's are)
//     planar_pva       xgc.planar_pva/1             -> unicycle_reference_trajectory_msgs/PlanarPvaReference
//     sim_pose         xgc.pose/1                   -> geometry_msgs/PoseStamped (frame `frame_id`)
//     sim_velocity     xgc.twist/1                  -> geometry_msgs/TwistStamped (frame `frame_id`)
//     sim_imu          xgc.imu/1                     -> sensor_msgs/Imu (orientation unknown)
//     sim_fcu_state    xgc.fcu_state/1               -> mavros_msgs/State
//   virtual MAVROS service facade -> module output
//     sim_fcu_request  xgc.fcu_request/2; `_topic` is a virtual MAVROS namespace.
//                      CommandBool and CommandLong ARM400 await an executed
//                      sim_fcu_result; SetMode reports only host delivery.
//     sim_fcu_result   xgc.fcu_result/1, shared batch EVENT input for the facade.
//     sim_extended_state xgc.fcu_extended_state/1 -> mavros_msgs/ExtendedState.
//
// Threading: ROS callbacks run on this plugin's own queue. Each step first
// publishes what the modules wrote since the last step, then services that
// queue for a steady-time budget. The budget is fixed at step entry, so a
// paused Session clock cannot keep the step waiting. ros::init is once per
// process and is shared with the clock-source service (same node name and
// the process ROS master). A clock source that has not been started leaves
// the output gate unclaimed, which is the existing wall path.
//
// Config: `<port>_topic` (string), `node_name`, `frame_id` (default "world"),
// `queue_size` (default 10), `slice_ms` (default 0: service ROS until the
// round's deadline). With long rounds (100 ms) set `slice_ms` (e.g. 2)
// and the plugin's `wake_ms` to the same value: each step then services ROS
// for at most one slice, and module outputs reach ROS within about a slice
// instead of a round. A slice
// remainder shorter than the kernel's timer slack (50 us) is not waited, so
// `slice_ms` = 0.001 is exactly one non-blocking pass (ros_slice.hpp).
// `sim_mocap_position_stddev_m = [x,y,z]` enables publication-side mocap
// position measurement noise on sim_pose only; absent means exact plant pose.
// `sim_mocap_noise_seed` selects a reproducible per-robot stream (default 1).

#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <exception>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <hover_thrust_estimator_msgs/HoverThrustEstimate.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <std_msgs/Float64MultiArray.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include "sim_odometry.hpp"
#include "sim_imu.hpp"
#include "attitude_target_full.hpp"
#include "sim_mocap.hpp"
#include "sim_fcu_rpc.hpp"
#include "sim_provider_rpc.hpp"
#include <xgc2_lightweight_sim_msgs/SetProvider.h>
#include <multirotor_reference_trajectory_msgs/AnalyticReference.h>
#include <multirotor_reference_trajectory_msgs/ReferenceStatus.h>
#include <multirotor_reference_trajectory_msgs/SampledReference.h>
#include <rigid_state_estimator_msgs/RigidStateEstimate.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <unicycle_reference_trajectory_msgs/PlanarPvaReference.h>

#include <flat_config.hpp>
#include <multirotor_reference_trajectory/reference_wire.hpp>
#include "ros_edge.hpp"
#include "ros_slice.hpp"
#include "xgc_rt.h"
#include <xgc-robotics-interfaces/robotics_interfaces_v1.h>
#include <xgc-lightweight-sim/simulation_records_v1.h>
#include <estimator_vrpn_px4_rotor_state/native/rigid_state_wire_v1.h>
#include <hover_thrust_estimator/native/hover_thrust_wire.h>

#include <chrono>

namespace {

namespace cfg = xgc_rt_config;

enum Port : uint32_t {
  kImu = 0,
  kPose,
  kAttitudeTarget,
  kVisionPose,
  kRigidStateEstimate,
  kFcuState,
  kLocalPose,
  kLocalVelocity,
  kFcuImu,
  kBattery,
  kCommand,
  kAlgSetpoint,
  kSetpoint,
  kAttitudeRate,
  kStatus,
  kFcuRequest,
  kRefAnalytic,
  kRefSampled,
  kRefReset,
  kRefStatus,
  kRefActiveAnalytic,
  kRefActiveSampled,
  kHoverThrust,
  kControllerState,
  kPlanarPva,
  kSimPose,
  kSimVelocity,
  kSimImu,
  kSimFcuState,
  kCmdVel,
  kSimFcuRequest,
  kAttitudeTargetFull,
  kSimAttitudeTarget,
  kSimHoverThrust,
  kSimFcuResult,
  kSimExtendedState,
  kSimProviderRequest,
  kSimProviderResult,
  kPortCount
};

const char* const kPortNames[kPortCount] = {
    "imu",
    "pose",
    "attitude_target",
    "vision_pose",
    "rigid_state_estimate",
    "fcu_state",
    "local_pose",
    "local_velocity",
    "fcu_imu",
    "battery",
    "command",
    "alg_setpoint",
    "setpoint",
    "attitude_rate",
    "status",
    "fcu_request",
    "ref_analytic",
    "ref_sampled",
    "ref_reset",
    "ref_status",
    "ref_active_analytic",
    "ref_active_sampled",
    "hover_thrust",
    "controller_state",
    "planar_pva",
    "sim_pose",
    "sim_velocity",
    "sim_imu",
    "sim_fcu_state",
    "cmd_vel",
    "sim_fcu_request",
    "attitude_target_full",
    "sim_attitude_target",
    "sim_hover_thrust",
    "sim_fcu_result",
    "sim_extended_state",
    "sim_provider_request",
    "sim_provider_result",
};

// All simulated FCU services in this process share one low-frequency queue.
// Its callbacks only use the thread-safe RPC helper, never RosIo or the Host.
struct SimServiceLoop {
  ros::CallbackQueue queue;
  ros::AsyncSpinner spinner{1, &queue};
  SimServiceLoop() { spinner.start(); }
  ~SimServiceLoop() { spinner.stop(); }
};

std::shared_ptr<SimServiceLoop> shared_sim_service_loop() {
  static const auto loop = std::make_shared<SimServiceLoop>();
  return loop;
}

std::shared_ptr<SimServiceLoop> shared_provider_service_loop() {
  // A provider stop must be able to finish while the FCU queue awaits an ACK.
  static const auto loop = std::make_shared<SimServiceLoop>();
  return loop;
}

double stamp_or_now(const ros::Time& t) { return (t.isZero() ? ros::Time::now() : t).toSec(); }

void quat(double* wxyz, const geometry_msgs::Quaternion& q) {
  wxyz[0] = q.w;
  wxyz[1] = q.x;
  wxyz[2] = q.y;
  wxyz[3] = q.z;
}

void vec3(double* out, const geometry_msgs::Vector3& v) {
  out[0] = v.x;
  out[1] = v.y;
  out[2] = v.z;
}

void vec3(geometry_msgs::Vector3& out, const double* in) {
  out.x = in[0];
  out.y = in[1];
  out.z = in[2];
}

// Copies a std::string into a fixed char field, always NUL-terminated.
template <size_t N>
void text(char (&out)[N], const std::string& in) {
  const size_t n = std::min(in.size(), N - 1);
  std::memcpy(out, in.data(), n);
  out[n] = '\0';
}

template <size_t N>
std::string text(const char (&in)[N]) {
  return std::string(in, strnlen(in, N));
}

struct RosIo {
  const xgc_host_api* host;
  std::string topics[kPortCount];
  std::string sim_odometry_topic;
  std::string sim_odometry_child_frame;
  std::string sim_body_pose_topic;
  ros::Publisher sim_body_pose_pub;
  xgc_pose_v1 sim_pose_cache{};
  std::unique_ptr<SimMocapMeasurement> sim_mocap;
  xgc_twist_v1 sim_velocity_cache{};
  xgc_imu_v1 sim_imu_cache{};
  bool have_sim_imu{false};
  bool sim_imu_orientation_from_pose{false};
  double last_sim_imu_stamp{0};
  bool have_sim_pose{false}, have_sim_velocity{false};
  double last_sim_odometry_stamp{0};
  ros::Publisher sim_odometry_pub;
  std::string sim_hover_thrust_trace_topic;
  ros::Publisher sim_hover_thrust_trace_pub;
  std::string node_name{"xgc_ros_io"};
  std::string frame_id{"world"};
  int queue_size{10};
  double slice_ms{0.0};
  ros::CallbackQueue queue;
  std::unique_ptr<ros::NodeHandle> nh;
  std::vector<ros::Subscriber> subs;
  ros::Publisher pubs[kPortCount];
  ros::ServiceServer sim_command_service;
  ros::ServiceServer sim_arming_service;
  ros::ServiceServer sim_set_mode_service;
  std::shared_ptr<xgc_sim_fcu::Rpc> sim_rpc;
  std::shared_ptr<SimServiceLoop> sim_service_loop;
  std::string sim_provider_service;
  std::string sim_provider_group;
  bool sim_provider_request_owner{true};
  std::shared_ptr<xgc_sim_provider::Rpc> provider_rpc;
  std::shared_ptr<SimServiceLoop> provider_service_loop;
  ros::ServiceServer provider_service;
  ros::Subscriber provider_pva_sub, provider_attitude_sub;
  xgc_sim_provider::State provider_state;
  uint64_t provider_controls_epoch{0};
  int sim_fcu_robot_index{0};
  double last_sim_extended_stamp{-1};
  uint64_t round{0};
  uint64_t from_ros{0};
  uint64_t to_ros{0};
  bool write_failed{false};

  // fcu_request: service calls run in order on `caller`; results come back as
  // log lines, logged from the plugin thread.
  std::thread caller;
  std::mutex calls_mutex;
  std::condition_variable calls_cv;
  std::deque<xgc_fcu_request_v1> calls;
  std::vector<std::string> call_log;
  bool calls_stop{false};

  ~RosIo() {
    stop_provider_services();
    stop_sim_services();
    stop_calls();
    release_publishers();
  }

  // Unadvertises every publisher of this edge and releases its NodeHandle.
  // roscpp's TopicManager::unadvertise is not safe against a concurrent
  // advertise or unadvertise in the same process (it erases with an iterator
  // from an earlier critical section, so it can remove another topic's
  // Publication, and the next publish on that topic faults), so all edges of a
  // process take one mutex for it (ros_publisher_lifecycle_test.cpp).
  void release_publishers() {
    std::lock_guard<std::mutex> lifecycle(xgc_ros_edge::publisher_lifecycle_mutex());
    sim_odometry_pub.shutdown();
    sim_body_pose_pub.shutdown();
    sim_hover_thrust_trace_pub.shutdown();
    for (auto& p : pubs) p.shutdown();
    nh.reset();
  }













  void log(xgc_log_level level, const std::string& m) const { host->log(host->host, level, m.c_str()); }

  bool enabled(Port p) const { return !topics[p].empty(); }

  template <typename T>
  bool write(Port p, const T& payload) {
    return write_bytes(p, reinterpret_cast<const uint8_t*>(&payload), sizeof payload);
  }

  bool write_bytes(Port p, const uint8_t* data, size_t len) {
    const bool ok = host->publish(host->host, p, round, data, static_cast<uint32_t>(len)) == XGC_OK;
    if (!ok) write_failed = true;
    ++from_ros;
    return ok;
  }

  // --- ROS -> module outputs ------------------------------------------------

  void on_imu(const sensor_msgs::Imu::ConstPtr& m) {
    xgc_imu_v1 s{};
    s.stamp = stamp_or_now(m->header.stamp);
    s.accel[0] = m->linear_acceleration.x;
    s.accel[1] = m->linear_acceleration.y;
    s.accel[2] = m->linear_acceleration.z;
    s.gyro[0] = m->angular_velocity.x;
    s.gyro[1] = m->angular_velocity.y;
    s.gyro[2] = m->angular_velocity.z;
    write(kImu, s);
  }

  void on_pose(const geometry_msgs::PoseStamped::ConstPtr& m) {
    xgc_pose_v1 s{};
    s.stamp = stamp_or_now(m->header.stamp);
    s.position[0] = m->pose.position.x;
    s.position[1] = m->pose.position.y;
    s.position[2] = m->pose.position.z;
    quat(s.q_wxyz, m->pose.orientation);
    write(kPose, s);
  }

  void on_attitude_target(const mavros_msgs::AttitudeTarget::ConstPtr& m) {
    xgc_attitude_target_v1 s{};
    s.stamp = stamp_or_now(m->header.stamp);
    quat(s.q_wxyz, m->orientation);
    s.thrust = m->thrust;
    s.ignore_thrust = (m->type_mask & mavros_msgs::AttitudeTarget::IGNORE_THRUST) ? 1u : 0u;
    write(kAttitudeTarget, s);
  }

  void on_attitude_target_full(const mavros_msgs::AttitudeTarget::ConstPtr& m) {
    const auto sample = full_attitude_target(*m, stamp_or_now(m->header.stamp));
    write(kAttitudeTargetFull, sample);
  }



  void on_fcu_state(const mavros_msgs::State::ConstPtr& m) {
    xgc_fcu_state_v1 s{};
    s.stamp = stamp_or_now(m->header.stamp);
    s.connected = m->connected ? 1u : 0u;
    s.armed = m->armed ? 1u : 0u;
    s.guided = m->guided ? 1u : 0u;
    s.manual_input = m->manual_input ? 1u : 0u;
    s.system_status = m->system_status;
    text(s.mode, m->mode);
    write(kFcuState, s);
  }

  void on_local_pose(const geometry_msgs::PoseStamped::ConstPtr& m) {
    xgc_pose_v1 s{};
    s.stamp = stamp_or_now(m->header.stamp);
    s.position[0] = m->pose.position.x;
    s.position[1] = m->pose.position.y;
    s.position[2] = m->pose.position.z;
    quat(s.q_wxyz, m->pose.orientation);
    write(kLocalPose, s);
  }

  void on_local_velocity(const geometry_msgs::TwistStamped::ConstPtr& m) {
    xgc_twist_v1 s{};
    s.stamp = stamp_or_now(m->header.stamp);
    vec3(s.linear, m->twist.linear);
    vec3(s.angular, m->twist.angular);
    write(kLocalVelocity, s);
  }

  void on_fcu_imu(const sensor_msgs::Imu::ConstPtr& m) {
    xgc_imu_v1 s{};
    s.stamp = stamp_or_now(m->header.stamp);
    vec3(s.accel, m->linear_acceleration);
    vec3(s.gyro, m->angular_velocity);
    write(kFcuImu, s);
  }

  void on_battery(const sensor_msgs::BatteryState::ConstPtr& m) {
    xgc_battery_v1 s{};
    s.stamp = stamp_or_now(m->header.stamp);
    s.voltage = m->voltage;
    s.percentage = m->percentage;
    write(kBattery, s);
  }

  void on_command(const std_msgs::String::ConstPtr& m) {
    xgc_command_v1 s{};
    text(s.text, m->data);
    write(kCommand, s);
  }

  void on_cmd_vel(const geometry_msgs::Twist::ConstPtr& m) {
    xgc_twist_v1 s{};
    s.stamp = ros::Time::now().toSec();
    vec3(s.linear, m->linear);
    vec3(s.angular, m->angular);
    write(kCmdVel, s);
  }

  void on_alg_setpoint(const mavros_msgs::PositionTarget::ConstPtr& m) {
    xgc_position_target_v1 s{};
    s.stamp = stamp_or_now(m->header.stamp);
    s.position[0] = m->position.x;
    s.position[1] = m->position.y;
    s.position[2] = m->position.z;
    s.velocity[0] = m->velocity.x;
    s.velocity[1] = m->velocity.y;
    s.velocity[2] = m->velocity.z;
    s.acceleration[0] = m->acceleration_or_force.x;
    s.acceleration[1] = m->acceleration_or_force.y;
    s.acceleration[2] = m->acceleration_or_force.z;
    s.yaw = m->yaw;
    s.yaw_rate = m->yaw_rate;
    s.type_mask = m->type_mask;
    s.coordinate_frame = m->coordinate_frame;
    write(kAlgSetpoint, s);
  }

  void on_ref_analytic(const multirotor_reference_trajectory_msgs::AnalyticReference::ConstPtr& m) {
    const auto bytes = xgc_ref_wire::encode_analytic(*m);
    write_bytes(kRefAnalytic, bytes.data(), bytes.size());
  }


  void on_ref_sampled(const multirotor_reference_trajectory_msgs::SampledReference::ConstPtr& m) {
    const auto bytes = xgc_ref_wire::encode_sampled(*m);
    write_bytes(kRefSampled, bytes.data(), bytes.size());
  }

  void on_ref_reset(const std_msgs::Empty::ConstPtr&) { write(kRefReset, xgc_ref_reset_v1{}); }

  void on_controller_state(const std_msgs::String::ConstPtr& m) {
    xgc_controller_status_v1 s{};
    s.stamp = ros::Time::now().toSec();
    text(s.state, m->data);
    write(kControllerState, s);
  }

  void on_hover_thrust(const hover_thrust_estimator_msgs::HoverThrustEstimate::ConstPtr& m) {
    hover_thrust_native::xgc_hover_thrust_v1 s{};
    s.stamp = stamp_or_now(m->header.stamp);
    s.hover_thrust = m->hover_thrust;
    s.state = m->state;
    s.flags = m->flags;
    write(kHoverThrust, s);
  }

  // --- fcu_request service calls -------------------------------------------

  void call_loop() {
    const std::string ns = topics[kFcuRequest];
    ros::ServiceClient command = nh->serviceClient<mavros_msgs::CommandLong>(ns + "/cmd/command", true);
    ros::ServiceClient set_mode = nh->serviceClient<mavros_msgs::SetMode>(ns + "/set_mode", true);
    for (;;) {
      xgc_fcu_request_v1 r;
      {
        std::unique_lock<std::mutex> lock(calls_mutex);
        calls_cv.wait(lock, [&] { return calls_stop || !calls.empty(); });
        if (calls_stop) return;
        r = calls.front();
        calls.pop_front();
      }
      std::string result;
      if (!xgc_ros_edge::output_allowed()) {
        result = "fcu_request discarded while the output gate is closed";
      } else if (r.kind == 1) {
        // A persistent client drops its connection on a failed call; reconnect.
        if (!command.isValid()) command = nh->serviceClient<mavros_msgs::CommandLong>(ns + "/cmd/command", true);
        mavros_msgs::CommandLong srv;
        srv.request.command = 400;  // MAV_CMD_COMPONENT_ARM_DISARM
        srv.request.param1 = r.arm ? 1.0 : 0.0;
        const bool ok = command.call(srv);
        result = std::string(r.arm ? "arm" : "disarm") + (ok ? (srv.response.success ? ": success" : ": refused") : ": call failed");
      } else if (r.kind == 2) {
        if (!set_mode.isValid()) set_mode = nh->serviceClient<mavros_msgs::SetMode>(ns + "/set_mode", true);
        mavros_msgs::SetMode srv;
        srv.request.custom_mode = text(r.mode);
        const bool ok = set_mode.call(srv);
        result = "set_mode " + srv.request.custom_mode + (ok ? (srv.response.mode_sent ? ": sent" : ": refused") : ": call failed");
      } else {
        result = "unknown fcu_request kind " + std::to_string(r.kind) + " dropped";
      }
      std::lock_guard<std::mutex> lock(calls_mutex);
      call_log.push_back("ros_io: " + result);
    }
  }

  void stop_calls() {
    if (!caller.joinable()) return;
    {
      std::lock_guard<std::mutex> lock(calls_mutex);
      calls_stop = true;
    }
    calls_cv.notify_all();
    caller.join();
  }

  static bool on_sim_command(const std::shared_ptr<xgc_sim_fcu::Rpc>& rpc, uint64_t epoch,
                            mavros_msgs::CommandLong::Request& request,
                            mavros_msgs::CommandLong::Response& response) {
    response.success = false;
    response.result = 4;
    if (request.command != 400) {
      response.result = 3;
      return true;
    }
    if (request.param1 != 0.0f && request.param1 != 1.0f) {
      response.result = 2;
      return true;
    }
    if (request.broadcast || request.confirmation != 0 ||
        (request.param2 != 0.0f && request.param2 != 21196.0f) || request.param3 != 0.0f ||
        request.param4 != 0.0f || request.param5 != 0.0f || request.param6 != 0.0f || request.param7 != 0.0f) {
      response.result = 3;  // Unsupported fields are not silently dropped.
      return true;
    }
    if (!xgc_ros_edge::output_allowed()) return true;
    xgc_fcu_request_v2 r{};
    r.stamp = ros::Time::now().toSec();
    r.kind = 1;
    r.arm = request.param1 == 1.0f ? 1u : 0u;
    r.flags = request.param2 == 21196.0f ? 1u : 0u;
    const auto reply = rpc->call(r, true, epoch);
    response.success = reply.has_result && reply.result == 0;
    response.result = reply.result;
    return true;
  }

  static bool on_sim_arming(const std::shared_ptr<xgc_sim_fcu::Rpc>& rpc, uint64_t epoch,
                           mavros_msgs::CommandBool::Request& request,
                           mavros_msgs::CommandBool::Response& response) {
    response.success = false;
    response.result = 4;
    if (!xgc_ros_edge::output_allowed()) return true;
    xgc_fcu_request_v2 r{};
    r.stamp = ros::Time::now().toSec();
    r.kind = 1;
    r.arm = request.value ? 1u : 0u;
    const auto reply = rpc->call(r, true, epoch);
    response.success = reply.has_result && reply.result == 0;
    response.result = reply.result;
    return true;
  }

  static bool on_sim_set_mode(const std::shared_ptr<xgc_sim_fcu::Rpc>& rpc, uint64_t epoch,
                             mavros_msgs::SetMode::Request& request, mavros_msgs::SetMode::Response& response) {
    response.mode_sent = false;
    if (!xgc_ros_edge::output_allowed() || request.base_mode != 0 || request.custom_mode.empty() ||
        request.custom_mode.size() >= sizeof(xgc_fcu_request_v2::mode) ||
        request.custom_mode.find('\0') != std::string::npos) return true;
    xgc_fcu_request_v2 r{};
    r.stamp = ros::Time::now().toSec();
    r.kind = 2;
    text(r.mode, request.custom_mode);
    response.mode_sent = rpc->call(r, false, epoch).sent;
    return true;
  }

  void stop_sim_services() {
    if (sim_rpc) sim_rpc->close();
    sim_arming_service.shutdown();
    sim_command_service.shutdown();
    sim_set_mode_service.shutdown();
    sim_service_loop.reset();
  }

  void bind_sim_fcu_services() {
    sim_command_service.shutdown();
    sim_arming_service.shutdown();
    sim_set_mode_service.shutdown();
    if (!sim_rpc || !enabled(kSimFcuRequest)) return;
    sim_service_loop = shared_sim_service_loop();
    const auto rpc = sim_rpc;
    const auto epoch = rpc->epoch();
    const std::string ns = topics[kSimFcuRequest];
    ros::AdvertiseServiceOptions command, arming, mode;
    command.init<mavros_msgs::CommandLong::Request, mavros_msgs::CommandLong::Response>(
        ns + "/cmd/command", [rpc, epoch](auto& request, auto& response) { return on_sim_command(rpc, epoch, request, response); });
    arming.init<mavros_msgs::CommandBool::Request, mavros_msgs::CommandBool::Response>(
        ns + "/cmd/arming", [rpc, epoch](auto& request, auto& response) { return on_sim_arming(rpc, epoch, request, response); });
    mode.init<mavros_msgs::SetMode::Request, mavros_msgs::SetMode::Response>(
        ns + "/set_mode", [rpc, epoch](auto& request, auto& response) { return on_sim_set_mode(rpc, epoch, request, response); });
    command.callback_queue = arming.callback_queue = mode.callback_queue = &sim_service_loop->queue;
    sim_command_service = nh->advertiseService(command);
    sim_arming_service = nh->advertiseService(arming);
    sim_set_mode_service = nh->advertiseService(mode);
  }

  void publish_sim_requests() {
    if (!sim_rpc) return;
    for (const auto& request : sim_rpc->take_requests()) {
      if (!xgc_ros_edge::output_allowed()) {
        sim_rpc->output_open(false);
        break;
      }
      if (!sim_rpc->can_publish(request.request_id)) continue;
      sim_rpc->published(request.request_id, write(kSimFcuRequest, request));
    }
  }

  static bool on_provider(const std::shared_ptr<xgc_sim_provider::Rpc>& rpc,
                          xgc2_lightweight_sim_msgs::SetProvider::Request& request, xgc2_lightweight_sim_msgs::SetProvider::Response& response) {
    const auto current = rpc->state();
    response.accepted = false;
    response.generation = current.generation;
    response.enabled = current.enabled;
    response.reason = 2;
    response.message = "invalid action";
    if (request.action > 2) return true;
    xgc_sim_provider_request_v1 wire{};
    wire.stamp = ros::Time::now().toSec();
    wire.action = request.action;
    wire.generation = request.generation;
    const auto reply = rpc->call(wire);
    if (!reply.received) return false;  // No fabricated model reason on timeout/closed transport.
    response.accepted = reply.result.accepted != 0;
    response.generation = reply.result.generation;
    response.enabled = reply.result.enabled != 0;
    response.reason = reply.result.reason;
    response.message = reply.result.reason == 0 ? "accepted" :
                       reply.result.reason == 1 ? "stale generation" : "invalid action or body";
    return true;
  }

  void bind_provider_controls() {
    ++provider_controls_epoch;
    provider_pva_sub.shutdown();
    provider_attitude_sub.shutdown();
    if (!provider_state.enabled) return;
    const auto generation = provider_state.generation;
    const auto epoch = provider_controls_epoch;
    if (enabled(kAlgSetpoint))
      provider_pva_sub = nh->subscribe<mavros_msgs::PositionTarget>(topics[kAlgSetpoint], queue_size,
          [this, generation, epoch](const mavros_msgs::PositionTarget::ConstPtr& message) {
            if (provider_state.enabled && provider_state.generation == generation && provider_controls_epoch == epoch)
              on_alg_setpoint(message);
          });
    if (enabled(kAttitudeTargetFull))
      provider_attitude_sub = nh->subscribe<mavros_msgs::AttitudeTarget>(topics[kAttitudeTargetFull], queue_size,
          [this, generation, epoch](const mavros_msgs::AttitudeTarget::ConstPtr& message) {
            if (provider_state.enabled && provider_state.generation == generation && provider_controls_epoch == epoch)
              on_attitude_target_full(message);
          });
  }

  void consume_provider_results() {
    if (!provider_rpc) return;
    xgc_sample_view view;
    while (host->next(host->host, kSimProviderResult, &view) == XGC_OK) {
      if (view.len != sizeof(xgc_sim_provider_result_v1)) continue;
      xgc_sim_provider_result_v1 result;
      std::memcpy(&result, view.data, sizeof result);
      if (!provider_rpc->result(result)) continue;
      const auto state = provider_rpc->state();
      if (state.generation == provider_state.generation && state.enabled == provider_state.enabled) continue;
      if (sim_rpc) sim_rpc->close();
      provider_state = state;
      bind_provider_controls();
      if (provider_state.enabled && sim_rpc) sim_rpc->open();
      bind_sim_fcu_services();
    }
  }

  void publish_provider_requests() {
    if (!provider_rpc) return;
    if (!sim_provider_group.empty()) {
      if (!sim_provider_request_owner) return;
      for (const auto& work : xgc_sim_provider::Groups::instance().take(sim_provider_group)) {
        if (!work.rpc->can_publish(work.request.request_id)) continue;
        work.rpc->published(work.request.request_id, write(kSimProviderRequest, work.request));
      }
    } else {
      for (const auto& request : provider_rpc->take_requests())
        if (provider_rpc->can_publish(request.request_id))
          provider_rpc->published(request.request_id, write(kSimProviderRequest, request));
    }
  }

  void stop_provider_services() {
    ++provider_controls_epoch;
    if (provider_rpc) provider_rpc->close();
    if (provider_rpc && !sim_provider_group.empty())
      xgc_sim_provider::Groups::instance().remove(sim_provider_group, static_cast<uint32_t>(sim_fcu_robot_index), provider_rpc);
    provider_service.shutdown();
    provider_pva_sub.shutdown();
    provider_attitude_sub.shutdown();
    provider_service_loop.reset();
  }

  void discard_pending_output() {
    xgc_sample_view v;
    for (uint32_t port = 0; port < kPortCount; ++port) {
      while (host->next(host->host, port, &v) == XGC_OK) {
      }
    }
    std::lock_guard<std::mutex> lock(calls_mutex);
    calls.clear();
  }

  // --- module inputs -> ROS -------------------------------------------------

  void forward_inputs() {
    if (!xgc_ros_edge::output_allowed()) {
      discard_pending_output();
      return;
    }
    xgc_sample_view v;
    while (host->next(host->host, kVisionPose, &v) == XGC_OK) {
      if (v.len != sizeof(xgc_pose_v1)) continue;
      xgc_pose_v1 s;
      std::memcpy(&s, v.data, sizeof s);
      geometry_msgs::PoseStamped m;
      m.header.stamp.fromSec(s.stamp);
      m.header.frame_id = frame_id;
      m.pose.position.x = s.position[0];
      m.pose.position.y = s.position[1];
      m.pose.position.z = s.position[2];
      m.pose.orientation.w = s.q_wxyz[0];
      m.pose.orientation.x = s.q_wxyz[1];
      m.pose.orientation.y = s.q_wxyz[2];
      m.pose.orientation.z = s.q_wxyz[3];
      pubs[kVisionPose].publish(m);
      ++to_ros;
    }
    while (host->next(host->host, kSimPose, &v) == XGC_OK) {
      if (v.len != sizeof(xgc_pose_v1)) continue;
      xgc_pose_v1 s;
      std::memcpy(&s, v.data, sizeof s);
      if (sim_body_pose_pub) {
        geometry_msgs::PoseStamped truth;
        truth.header.stamp.fromSec(s.stamp);
        truth.header.frame_id = "world";
        truth.pose.position.x = s.position[0];
        truth.pose.position.y = s.position[1];
        truth.pose.position.z = s.position[2];
        truth.pose.orientation.w = s.q_wxyz[0];
        truth.pose.orientation.x = s.q_wxyz[1];
        truth.pose.orientation.y = s.q_wxyz[2];
        truth.pose.orientation.z = s.q_wxyz[3];
        sim_body_pose_pub.publish(truth);
        ++to_ros;
      }
      // Only the explicitly configured mocap edge synthesizes a measurement.
      // Other consumers of this plant sample retain the unmodified truth.
      if (sim_mocap && !sim_mocap->sample(s, &s)) continue;
      geometry_msgs::PoseStamped m;
      m.header.stamp.fromSec(s.stamp);
      m.header.frame_id = frame_id;
      m.pose.position.x = s.position[0];
      m.pose.position.y = s.position[1];
      m.pose.position.z = s.position[2];
      m.pose.orientation.w = s.q_wxyz[0];
      m.pose.orientation.x = s.q_wxyz[1];
      m.pose.orientation.y = s.q_wxyz[2];
      m.pose.orientation.z = s.q_wxyz[3];
      pubs[kSimPose].publish(m);
      ++to_ros;
      sim_pose_cache = s;
      have_sim_pose = true;
    }
    while (host->next(host->host, kSimVelocity, &v) == XGC_OK) {
      if (v.len != sizeof(xgc_twist_v1)) continue;
      xgc_twist_v1 s;
      std::memcpy(&s, v.data, sizeof s);
      geometry_msgs::TwistStamped m;
      m.header.stamp.fromSec(s.stamp);
      m.header.frame_id = frame_id;
      vec3(m.twist.linear, s.linear);
      vec3(m.twist.angular, s.angular);
      pubs[kSimVelocity].publish(m);
      ++to_ros;
      sim_velocity_cache = s;
      have_sim_velocity = true;
    }
    if (sim_odometry_pub && have_sim_pose && have_sim_velocity) {
      SimOdometry measured;
      if (measured_sim_odometry(sim_pose_cache, sim_velocity_cache, last_sim_odometry_stamp, &measured)) {
        nav_msgs::Odometry m;
        m.header.stamp.fromSec(measured.pose.stamp);
        m.header.frame_id = frame_id;
        m.child_frame_id = sim_odometry_child_frame;
        m.pose.pose.position.x = measured.pose.position[0];
        m.pose.pose.position.y = measured.pose.position[1];
        m.pose.pose.position.z = measured.pose.position[2];
        m.pose.pose.orientation.w = measured.pose.q_wxyz[0];
        m.pose.pose.orientation.x = measured.pose.q_wxyz[1];
        m.pose.pose.orientation.y = measured.pose.q_wxyz[2];
        m.pose.pose.orientation.z = measured.pose.q_wxyz[3];
        vec3(m.twist.twist.linear, measured.linear);
        vec3(m.twist.twist.angular, measured.angular);
        sim_odometry_pub.publish(m);
        last_sim_odometry_stamp = measured.pose.stamp;
        ++to_ros;
      }
    }
    const auto publish_imu = [&](const xgc_imu_v1& s, const xgc_pose_v1* pose) {
      sensor_msgs::Imu m;
      m.header.stamp.fromSec(s.stamp);
      m.header.frame_id = sim_odometry_child_frame;
      if (pose) {
        m.orientation.w = pose->q_wxyz[0];
        m.orientation.x = pose->q_wxyz[1];
        m.orientation.y = pose->q_wxyz[2];
        m.orientation.z = pose->q_wxyz[3];
      } else {
        // A stand-alone imu/1 input has no attitude in its wire schema.
        m.orientation_covariance[0] = -1.0;
      }
      vec3(m.linear_acceleration, s.accel);
      vec3(m.angular_velocity, s.gyro);
      pubs[kSimImu].publish(m);
      ++to_ros;
    };
    while (host->next(host->host, kSimImu, &v) == XGC_OK) {
      if (v.len != sizeof(xgc_imu_v1)) continue;
      std::memcpy(&sim_imu_cache, v.data, sizeof sim_imu_cache);
      have_sim_imu = true;
      if (!sim_imu_orientation_from_pose) publish_imu(sim_imu_cache, nullptr);
    }
    // State-QoS inputs retain the newest sample. Never attach another model
    // step's quaternion, nor the commanded attitude, to measured IMU data.
    if (sim_imu_orientation_from_pose && have_sim_pose && have_sim_imu &&
        matched_sim_imu(sim_pose_cache, sim_imu_cache, last_sim_imu_stamp)) {
      publish_imu(sim_imu_cache, &sim_pose_cache);
      last_sim_imu_stamp = sim_imu_cache.stamp;
    }
    while (host->next(host->host, kSimAttitudeTarget, &v) == XGC_OK) {
      if (v.len != sizeof(xgc_attitude_target_v2)) continue;
      xgc_attitude_target_v2 sample;
      std::memcpy(&sample, v.data, sizeof sample);
      mavros_msgs::AttitudeTarget m;
      if (!assign_full_attitude_target(sample, &m)) continue;
      m.header.stamp.fromSec(sample.stamp);
      m.header.frame_id = frame_id;
      pubs[kSimAttitudeTarget].publish(m);
      ++to_ros;
    }
    while (host->next(host->host, kSimHoverThrust, &v) == XGC_OK) {
      if (v.len != sizeof(hover_thrust_native::xgc_hover_thrust_v1)) continue;
      hover_thrust_native::xgc_hover_thrust_v1 sample;
      std::memcpy(&sample, v.data, sizeof sample);
      hover_thrust_estimator_msgs::HoverThrustEstimate m;
      m.header.stamp.fromSec(sample.stamp);
      if (sample.state < 10 || sample.state > 12) continue;
      m.state = sample.state - 10; // native domain IDs -> original ROS enum
      m.flags = sample.flags;
      m.hover_thrust = sample.hover_thrust;
      pubs[kSimHoverThrust].publish(m);
      ++to_ros;
      if (sim_hover_thrust_trace_pub) {
        std_msgs::Float64MultiArray trace;
        trace.layout.dim.resize(1);
        trace.layout.dim[0].label = "xgc.hover_thrust/1";
        trace.layout.dim[0].size = trace.layout.dim[0].stride = 9;
        trace.data = {sample.stamp, sample.hover_thrust, sample.raw_hover_thrust,
                      sample.initial_hover_thrust, sample.thrust_to_acceleration,
                      sample.last_estimate_stamp, double(sample.state),
                      double(sample.flags), double(sample.sample_used)};
        sim_hover_thrust_trace_pub.publish(trace);
        ++to_ros;
      }
    }
    while (host->next(host->host, kSimFcuState, &v) == XGC_OK) {
      if (v.len != sizeof(xgc_fcu_state_v1)) continue;
      xgc_fcu_state_v1 s;
      std::memcpy(&s, v.data, sizeof s);
      if (sim_rpc) sim_rpc->state(s.stamp, s.connected != 0);
      mavros_msgs::State m;
      m.header.stamp.fromSec(s.stamp);
      m.connected = s.connected != 0;
      m.armed = s.armed != 0;
      m.guided = s.guided != 0;
      m.manual_input = s.manual_input != 0;
      m.system_status = s.system_status;
      m.mode = text(s.mode);
      pubs[kSimFcuState].publish(m);
      ++to_ros;
    }
    while (host->next(host->host, kSimFcuResult, &v) == XGC_OK) {
      if (!sim_rpc || v.len != sizeof(xgc_fcu_result_v1)) continue;
      xgc_fcu_result_v1 result;
      std::memcpy(&result, v.data, sizeof result);
      sim_rpc->result(result);
    }
    while (host->next(host->host, kSimExtendedState, &v) == XGC_OK) {
      if (!enabled(kSimExtendedState) || v.len != sizeof(xgc_fcu_extended_state_v1)) continue;
      xgc_fcu_extended_state_v1 state;
      std::memcpy(&state, v.data, sizeof state);
      if (!std::isfinite(state.stamp) || state.stamp < 0 || state.stamp <= last_sim_extended_stamp ||
          state.count > 6 || static_cast<uint32_t>(sim_fcu_robot_index) >= state.count) continue;
      mavros_msgs::ExtendedState message;
      message.header.stamp.fromSec(state.stamp);
      message.landed_state = state.landed_state[sim_fcu_robot_index];
      message.vtol_state = state.vtol_state[sim_fcu_robot_index];
      pubs[kSimExtendedState].publish(message);
      last_sim_extended_stamp = state.stamp;
      ++to_ros;
    }



    while (host->next(host->host, kPlanarPva, &v) == XGC_OK) {
      if (v.len != sizeof(xgc_planar_pva_v1)) continue;
      xgc_planar_pva_v1 p;
      std::memcpy(&p, v.data, sizeof p);
      unicycle_reference_trajectory_msgs::PlanarPvaReference m;
      m.header.stamp.fromSec(p.stamp);
      m.x = p.x;
      m.y = p.y;
      m.yaw = p.yaw;
      m.vx = p.vx;
      m.vy = p.vy;
      m.ax = p.ax;
      m.ay = p.ay;
      pubs[kPlanarPva].publish(m);
      ++to_ros;
    }
    while (host->next(host->host, kRigidStateEstimate, &v) == XGC_OK) {
      if (v.len != sizeof(xgc_rigid_state_estimate_v1)) continue;
      xgc_rigid_state_estimate_v1 e;
      std::memcpy(&e, v.data, sizeof e);
      rigid_state_estimator_msgs::RigidStateEstimate m;
      m.header.stamp.fromSec(e.stamp);
      m.estimator_state = e.estimator_state;
      m.flags = e.flags;
      m.position.x = e.position[0];
      m.position.y = e.position[1];
      m.position.z = e.position[2];
      auto vec = [](geometry_msgs::Vector3& out, const double* in) {
        out.x = in[0];
        out.y = in[1];
        out.z = in[2];
      };
      vec(m.velocity, e.velocity);
      m.orientation.w = e.q_wxyz[0];
      m.orientation.x = e.q_wxyz[1];
      m.orientation.y = e.q_wxyz[2];
      m.orientation.z = e.q_wxyz[3];
      vec(m.angular_velocity, e.angular_velocity);
      vec(m.linear_acceleration, e.linear_acceleration);
      vec(m.gravity, e.gravity);
      vec(m.accel_bias, e.accel_bias);
      m.vrpn_observation_state = e.vrpn_observation_state;
      m.filter_health = e.filter_health;
      m.last_pose_reject_reason = e.last_pose_reject_reason;
      m.last_pose_accepted = e.last_pose_accepted != 0;
      m.last_fused_pose_stamp_sec = e.last_fused_pose_stamp_sec;
      m.vrpn_innovation_window_chi_square = e.vrpn_innovation_window_chi_square;
      m.last_pose_position_innovation_norm_m = e.last_pose_position_innovation_norm_m;
      m.last_pose_orientation_innovation_norm_rad = e.last_pose_orientation_innovation_norm_rad;
      m.last_pose_mahalanobis_distance = e.last_pose_mahalanobis_distance;
      m.innovation_position_gate_m = e.innovation_position_gate_m;
      m.innovation_orientation_gate_rad = e.innovation_orientation_gate_rad;
      m.pose_nis_gate = e.pose_nis_gate;
      m.last_imu_sample_stamp_sec = e.last_imu_sample_stamp_sec;
      m.last_vrpn_pose_stamp_sec = e.last_vrpn_pose_stamp_sec;
      m.filter_inertial_stamp_sec = e.filter_inertial_stamp_sec;
      m.filter_pose_stamp_sec = e.filter_pose_stamp_sec;
      m.vrpn_consecutive_rejects = e.vrpn_consecutive_rejects;
      m.vrpn_consecutive_accepts = e.vrpn_consecutive_accepts;
      pubs[kRigidStateEstimate].publish(m);
      ++to_ros;
    }
    while (host->next(host->host, kSetpoint, &v) == XGC_OK) {
      if (v.len != sizeof(xgc_position_target_v1)) continue;
      xgc_position_target_v1 s;
      std::memcpy(&s, v.data, sizeof s);
      mavros_msgs::PositionTarget m;
      m.header.stamp.fromSec(s.stamp);
      m.header.frame_id = "map";
      m.coordinate_frame = s.coordinate_frame;
      m.type_mask = s.type_mask;
      m.position.x = s.position[0];
      m.position.y = s.position[1];
      m.position.z = s.position[2];
      m.velocity.x = s.velocity[0];
      m.velocity.y = s.velocity[1];
      m.velocity.z = s.velocity[2];
      m.acceleration_or_force.x = s.acceleration[0];
      m.acceleration_or_force.y = s.acceleration[1];
      m.acceleration_or_force.z = s.acceleration[2];
      m.yaw = static_cast<float>(s.yaw);
      m.yaw_rate = static_cast<float>(s.yaw_rate);
      pubs[kSetpoint].publish(m);
      ++to_ros;
    }
    while (host->next(host->host, kAttitudeRate, &v) == XGC_OK) {
      if (v.len != sizeof(xgc_body_rate_thrust_v1)) continue;
      xgc_body_rate_thrust_v1 s;
      std::memcpy(&s, v.data, sizeof s);
      mavros_msgs::AttitudeTarget m;
      m.header.stamp.fromSec(s.stamp);
      m.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
      m.orientation.w = 1.0;
      m.body_rate.x = s.body_rate[0];
      m.body_rate.y = s.body_rate[1];
      m.body_rate.z = s.body_rate[2];
      m.thrust = static_cast<float>(std::min(1.0, std::max(0.0, s.thrust)));
      pubs[kAttitudeRate].publish(m);
      ++to_ros;
    }
    while (host->next(host->host, kStatus, &v) == XGC_OK) {
      if (v.len != sizeof(xgc_controller_status_v1)) continue;
      xgc_controller_status_v1 s;
      std::memcpy(&s, v.data, sizeof s);
      std_msgs::String m;
      m.data = text(s.state);
      pubs[kStatus].publish(m);
      ++to_ros;
    }
    while (host->next(host->host, kFcuRequest, &v) == XGC_OK) {
      if (v.len != sizeof(xgc_fcu_request_v1)) continue;
      xgc_fcu_request_v1 r;
      std::memcpy(&r, v.data, sizeof r);
      {
        std::lock_guard<std::mutex> lock(calls_mutex);
        calls.push_back(r);
      }
      calls_cv.notify_one();
      ++to_ros;
    }
    while (host->next(host->host, kRefStatus, &v) == XGC_OK) {
      multirotor_reference_trajectory_msgs::ReferenceStatus m;
      if (!xgc_ref_wire::decode_status(v.data, v.len, m)) continue;
      pubs[kRefStatus].publish(m);
      ++to_ros;
    }
    while (host->next(host->host, kRefActiveAnalytic, &v) == XGC_OK) {
      multirotor_reference_trajectory_msgs::AnalyticReference m;
      if (!xgc_ref_wire::decode_analytic(v.data, v.len, m)) continue;
      pubs[kRefActiveAnalytic].publish(m);
      ++to_ros;
    }
    while (host->next(host->host, kRefActiveSampled, &v) == XGC_OK) {
      multirotor_reference_trajectory_msgs::SampledReference m;
      if (!xgc_ref_wire::decode_sampled(v.data, v.len, m)) continue;
      pubs[kRefActiveSampled].publish(m);
      ++to_ros;
    }
    std::vector<std::string> lines;
    {
      std::lock_guard<std::mutex> lock(calls_mutex);
      lines.swap(call_log);
    }
    for (const auto& l : lines) log(XGC_LOG_INFO, l);

  }

  xgc_status activate() {
    const xgc_ros_edge::RosInit ros_init = xgc_ros_edge::ensure_ros(node_name);
    if (!ros_init.ok) {
      log(XGC_LOG_ERROR, std::string("ros_io: ") + ros_init.error);
      return XGC_ERR;
    }
    // Publishers are advertised under the process lifecycle mutex (see
    // release_publishers); subscribers and services do not need it.
    std::lock_guard<std::mutex> lifecycle(xgc_ros_edge::publisher_lifecycle_mutex());
    nh = std::make_unique<ros::NodeHandle>();
    nh->setCallbackQueue(&queue);
    if (enabled(kImu)) subs.push_back(nh->subscribe(topics[kImu], queue_size, &RosIo::on_imu, this));
    if (enabled(kPose)) subs.push_back(nh->subscribe(topics[kPose], queue_size, &RosIo::on_pose, this));
    if (enabled(kAttitudeTarget))
      subs.push_back(nh->subscribe(topics[kAttitudeTarget], queue_size, &RosIo::on_attitude_target, this));
    if (enabled(kAttitudeTargetFull) && !provider_rpc)
      subs.push_back(nh->subscribe(topics[kAttitudeTargetFull], queue_size, &RosIo::on_attitude_target_full, this));
    if (enabled(kVisionPose)) pubs[kVisionPose] = nh->advertise<geometry_msgs::PoseStamped>(topics[kVisionPose], queue_size);
    if (sim_imu_orientation_from_pose && (!enabled(kSimPose) || sim_odometry_child_frame.empty()))
      throw std::invalid_argument("ros-io: oriented simulated IMU requires sim_pose and a body frame");
    if (enabled(kSimPose)) pubs[kSimPose] = nh->advertise<geometry_msgs::PoseStamped>(topics[kSimPose], queue_size);
    if (!sim_body_pose_topic.empty()) {
      if (!enabled(kSimPose)) throw std::invalid_argument("sim_body_pose_topic requires sim_pose input");
      sim_body_pose_pub = nh->advertise<geometry_msgs::PoseStamped>(sim_body_pose_topic, queue_size);
    }
    if (enabled(kSimVelocity))
      pubs[kSimVelocity] = nh->advertise<geometry_msgs::TwistStamped>(topics[kSimVelocity], queue_size);
    if (!sim_odometry_topic.empty()) {
      if (!enabled(kSimPose) || !enabled(kSimVelocity) || sim_odometry_child_frame.empty())
        throw std::invalid_argument("sim_odometry_topic requires sim_pose, sim_velocity and a child frame");
      sim_odometry_pub = nh->advertise<nav_msgs::Odometry>(sim_odometry_topic, queue_size);
    }
    if (enabled(kSimImu)) pubs[kSimImu] = nh->advertise<sensor_msgs::Imu>(topics[kSimImu], queue_size);
    if (enabled(kSimAttitudeTarget))
      pubs[kSimAttitudeTarget] = nh->advertise<mavros_msgs::AttitudeTarget>(topics[kSimAttitudeTarget], queue_size);
    if (enabled(kSimHoverThrust))
      pubs[kSimHoverThrust] = nh->advertise<hover_thrust_estimator_msgs::HoverThrustEstimate>(topics[kSimHoverThrust], queue_size);
    if (!sim_hover_thrust_trace_topic.empty()) {
      if (!enabled(kSimHoverThrust))
        throw std::invalid_argument("sim_hover_thrust_trace_topic requires sim_hover_thrust_topic");
      sim_hover_thrust_trace_pub = nh->advertise<std_msgs::Float64MultiArray>(sim_hover_thrust_trace_topic, queue_size);
    }
    if (enabled(kSimFcuState)) pubs[kSimFcuState] = nh->advertise<mavros_msgs::State>(topics[kSimFcuState], queue_size);
    if (enabled(kSimExtendedState))
      pubs[kSimExtendedState] = nh->advertise<mavros_msgs::ExtendedState>(topics[kSimExtendedState], queue_size);
    if (enabled(kRigidStateEstimate))
      pubs[kRigidStateEstimate] =
          nh->advertise<rigid_state_estimator_msgs::RigidStateEstimate>(topics[kRigidStateEstimate], queue_size);
    if (enabled(kFcuState)) subs.push_back(nh->subscribe(topics[kFcuState], queue_size, &RosIo::on_fcu_state, this));
    if (enabled(kLocalPose)) subs.push_back(nh->subscribe(topics[kLocalPose], queue_size, &RosIo::on_local_pose, this));
    if (enabled(kLocalVelocity))
      subs.push_back(nh->subscribe(topics[kLocalVelocity], queue_size, &RosIo::on_local_velocity, this));
    if (enabled(kFcuImu)) subs.push_back(nh->subscribe(topics[kFcuImu], queue_size, &RosIo::on_fcu_imu, this));
    if (enabled(kBattery)) subs.push_back(nh->subscribe(topics[kBattery], queue_size, &RosIo::on_battery, this));
    if (enabled(kCommand)) subs.push_back(nh->subscribe(topics[kCommand], queue_size, &RosIo::on_command, this));
    if (enabled(kCmdVel)) subs.push_back(nh->subscribe(topics[kCmdVel], queue_size, &RosIo::on_cmd_vel, this));
    if (enabled(kAlgSetpoint) && !provider_rpc)
      subs.push_back(nh->subscribe(topics[kAlgSetpoint], queue_size, &RosIo::on_alg_setpoint, this));
    if (enabled(kSetpoint)) pubs[kSetpoint] = nh->advertise<mavros_msgs::PositionTarget>(topics[kSetpoint], queue_size);
    if (enabled(kAttitudeRate))
      pubs[kAttitudeRate] = nh->advertise<mavros_msgs::AttitudeTarget>(topics[kAttitudeRate], queue_size);
    if (enabled(kStatus)) pubs[kStatus] = nh->advertise<std_msgs::String>(topics[kStatus], queue_size);
    if (enabled(kFcuRequest)) caller = std::thread([this] { call_loop(); });
    if (enabled(kSimFcuRequest)) {
      if (!enabled(kSimFcuState) || !sim_rpc)
        throw std::invalid_argument("sim_fcu_request requires a configured sim_fcu_state feedback port");
      sim_rpc->open();
      if (provider_rpc) sim_rpc->close();
      bind_sim_fcu_services();
    }
    if (provider_rpc) {
      provider_state = {};
      provider_rpc->open();
      if (!sim_provider_group.empty())
        xgc_sim_provider::Groups::instance().add(sim_provider_group, static_cast<uint32_t>(sim_fcu_robot_index), provider_rpc);
      provider_service_loop = shared_provider_service_loop();
      const auto rpc = provider_rpc;
      ros::AdvertiseServiceOptions provider;
      provider.init<xgc2_lightweight_sim_msgs::SetProvider::Request, xgc2_lightweight_sim_msgs::SetProvider::Response>(
          sim_provider_service, [rpc](auto& request, auto& response) { return on_provider(rpc, request, response); });
      provider.callback_queue = &provider_service_loop->queue;
      provider_service = nh->advertiseService(provider);
      bind_provider_controls();
    }
    if (enabled(kRefAnalytic))
      subs.push_back(nh->subscribe(topics[kRefAnalytic], queue_size, &RosIo::on_ref_analytic, this));
    if (enabled(kRefSampled))
      subs.push_back(nh->subscribe(topics[kRefSampled], queue_size, &RosIo::on_ref_sampled, this));
    if (enabled(kControllerState))
      subs.push_back(nh->subscribe(topics[kControllerState], queue_size, &RosIo::on_controller_state, this));

    if (enabled(kPlanarPva))
      pubs[kPlanarPva] =
          nh->advertise<unicycle_reference_trajectory_msgs::PlanarPvaReference>(topics[kPlanarPva], queue_size);
    if (enabled(kHoverThrust))
      subs.push_back(nh->subscribe(topics[kHoverThrust], queue_size, &RosIo::on_hover_thrust, this));
    if (enabled(kRefReset)) subs.push_back(nh->subscribe(topics[kRefReset], queue_size, &RosIo::on_ref_reset, this));
    namespace rmsg = multirotor_reference_trajectory_msgs;
    if (enabled(kRefStatus)) pubs[kRefStatus] = nh->advertise<rmsg::ReferenceStatus>(topics[kRefStatus], queue_size, true);
    if (enabled(kRefActiveAnalytic))
      pubs[kRefActiveAnalytic] = nh->advertise<rmsg::AnalyticReference>(topics[kRefActiveAnalytic], queue_size, true);
    if (enabled(kRefActiveSampled))
      pubs[kRefActiveSampled] = nh->advertise<rmsg::SampledReference>(topics[kRefActiveSampled], queue_size, true);
    return XGC_OK;
  }

  xgc_status step(const xgc_step_ctx* ctx) {
    round = ctx->round;
    consume_provider_results();
    const bool output_open = xgc_ros_edge::output_allowed() && !xgc_ros_edge::take_suppress_backlog();
    if (sim_rpc) sim_rpc->output_open(output_open);
    if (!output_open) {
      discard_pending_output();
    } else {
      forward_inputs();
    }
    // One non-blocking pass, then a budget fixed in steady time. Session time
    // is not read again: a frozen simulation clock must not extend the wait.
    // A remainder shorter than the timer slack is not waited (ros_slice.hpp).
    queue.callAvailable(ros::WallDuration(0));
    const int64_t budget_ns = xgc_ros_slice::budget_ns(host->now(host->host), ctx->deadline, slice_ms);
    const auto started = std::chrono::steady_clock::now();
    while (ros::ok()) {
      const int64_t elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(
                                  std::chrono::steady_clock::now() - started)
                                  .count();
      const int64_t wait_ns = xgc_ros_slice::next_wait_ns(budget_ns, elapsed);
      if (wait_ns == 0) break;
      queue.callAvailable(ros::WallDuration(wait_ns * 1e-9));
    }
    publish_sim_requests();
    publish_provider_requests();
    if (write_failed) {
      log(XGC_LOG_ERROR, "ros_io: a module output write failed");
      return XGC_ERR;
    }
    return XGC_OK;
  }

  void shutdown() {
    stop_provider_services();
    stop_sim_services();
    stop_calls();
    sim_command_service.shutdown();
    sim_set_mode_service.shutdown();
    subs.clear();
    release_publishers();
  }
};

template <typename F>
xgc_status guarded(const xgc_host_api* host, const char* where, F&& f) {
  try {
    return f();
  } catch (const std::exception& e) {
    host->log(host->host, XGC_LOG_ERROR, (std::string(where) + ": " + e.what()).c_str());
    return XGC_ERR;
  } catch (...) {
    host->log(host->host, XGC_LOG_ERROR, (std::string(where) + ": unknown exception").c_str());
    return XGC_ERR;
  }
}

void* create(const xgc_host_api* host) {
  try {
    auto* self = new RosIo{};
    self->host = host;
    return self;
  } catch (...) {
    return nullptr;
  }
}

xgc_status configure(void* p, const char* config) {
  auto* self = static_cast<RosIo*>(p);
  return guarded(self->host, "configure", [&] {
    const std::string t = config ? config : "";
    for (uint32_t i = 0; i < kPortCount; ++i) self->topics[i] = cfg::text_or(t, (std::string(kPortNames[i]) + "_topic").c_str(), "");
    self->sim_odometry_topic = cfg::text_or(t, "sim_odometry_topic", "");
    self->sim_odometry_child_frame = cfg::text_or(t, "sim_odometry_child_frame", "base_link");
    self->sim_body_pose_topic = cfg::text_or(t, "sim_body_pose_topic", "");
    self->sim_hover_thrust_trace_topic = cfg::text_or(t, "sim_hover_thrust_trace_topic", "");
    self->sim_provider_service = cfg::text_or(t, "sim_provider_service", "");
    self->sim_provider_group = cfg::text_or(t, "sim_provider_group", "");
    self->sim_provider_request_owner = true;
    if (!cfg::boolean(t, "sim_provider_request_owner", &self->sim_provider_request_owner))
      throw std::invalid_argument("sim_provider_request_owner must be boolean");
    double rpc_index = 0, rpc_timeout_ms = 1000, rpc_freshness_ms = 500;
    if (!cfg::number(t, "sim_fcu_robot_index", &rpc_index) || !std::isfinite(rpc_index) ||
        rpc_index < 0 || rpc_index >= 6 || std::floor(rpc_index) != rpc_index ||
        !cfg::number(t, "sim_fcu_timeout_ms", &rpc_timeout_ms) || !std::isfinite(rpc_timeout_ms) ||
        rpc_timeout_ms < 1 || rpc_timeout_ms > 5000 || std::floor(rpc_timeout_ms) != rpc_timeout_ms ||
        !cfg::number(t, "sim_fcu_freshness_ms", &rpc_freshness_ms) || !std::isfinite(rpc_freshness_ms) ||
        rpc_freshness_ms < 1 || rpc_freshness_ms > 5000 || std::floor(rpc_freshness_ms) != rpc_freshness_ms)
      throw std::invalid_argument("invalid sim FCU index or RPC deadlines");
    self->sim_fcu_robot_index = static_cast<int>(rpc_index);
    if (self->enabled(kSimFcuRequest)) {
      self->topics[kSimFcuResult] = "fcu-result";  // Host-only batch result, no ROS topic.
      self->sim_rpc = std::make_shared<xgc_sim_fcu::Rpc>(
          static_cast<uint32_t>(self->sim_fcu_robot_index), std::chrono::milliseconds(static_cast<int>(rpc_timeout_ms)),
          std::chrono::milliseconds(static_cast<int>(rpc_freshness_ms)));
    } else self->sim_rpc.reset();
    if (!self->sim_provider_service.empty()) {
      self->provider_rpc = std::make_shared<xgc_sim_provider::Rpc>(static_cast<uint32_t>(self->sim_fcu_robot_index));
      self->topics[kSimProviderRequest] = self->topics[kSimProviderResult] = "provider";
    } else self->provider_rpc.reset();
    if (!cfg::boolean(t, "sim_imu_orientation_from_pose", &self->sim_imu_orientation_from_pose))
      throw std::invalid_argument("ros-io: sim_imu_orientation_from_pose must be boolean");
    self->node_name = cfg::text_or(t, "node_name", "xgc_ros_io");
    self->frame_id = cfg::text_or(t, "frame_id", "world");
    std::string noise_config;
    if (cfg::value(t, "sim_mocap_position_stddev_m", &noise_config)) {
      std::array<double, 3> stddev{};
      double seed = 1.0;
      if (self->topics[kSimPose].empty() || !self->sim_odometry_topic.empty() ||
          !cfg::numbers(t, "sim_mocap_position_stddev_m", stddev.data(), stddev.size()) ||
          !cfg::number(t, "sim_mocap_noise_seed", &seed) ||
          !std::isfinite(seed) || seed < 0.0 || seed > UINT32_MAX || std::floor(seed) != seed)
        throw std::invalid_argument("sim mocap requires sim_pose, no sim_odometry, and valid noise parameters");
      self->sim_mocap = std::make_unique<SimMocapMeasurement>(stddev, static_cast<uint32_t>(seed));
    } else {
      if (cfg::value(t, "sim_mocap_noise_seed", &noise_config))
        throw std::invalid_argument("sim_mocap_noise_seed requires sim_mocap_position_stddev_m");
      self->sim_mocap.reset();
    }
    if (!cfg::number(t, "slice_ms", &self->slice_ms) || self->slice_ms < 0.0) {
      self->log(XGC_LOG_ERROR, "ros_io: invalid slice_ms");
      return XGC_ERR;
    }
    if (!cfg::integer(t, "queue_size", &self->queue_size) || self->queue_size <= 0) {
      self->log(XGC_LOG_ERROR, "ros_io: invalid queue_size");
      return XGC_ERR;
    }
    return XGC_OK;
  });
}

xgc_status activate(void* p) {
  auto* self = static_cast<RosIo*>(p);
  return guarded(self->host, "activate", [&] { return self->activate(); });
}

xgc_status step(void* p, const xgc_step_ctx* ctx) {
  auto* self = static_cast<RosIo*>(p);
  return guarded(self->host, "step", [&] { return self->step(ctx); });
}

xgc_status deactivate(void* p) {
  auto* self = static_cast<RosIo*>(p);
  return guarded(self->host, "deactivate", [&] {
    self->shutdown();
    return XGC_OK;
  });
}

void destroy(void* p) { delete static_cast<RosIo*>(p); }

const char* domain_state(void* p) {
  const auto* self = static_cast<RosIo*>(p);
  if (!ros::isInitialized() || !ros::ok()) return "down";
  return self->from_ros || self->to_ros ? "flowing" : "connected";
}

const xgc_port_decl kPorts[kPortCount] = {
    {"imu", XGC_PORT_OUT_OPTIONAL, "xgc.imu/1", XGC_QOS_STATE},
    {"pose", XGC_PORT_OUT_OPTIONAL, "xgc.pose/1", XGC_QOS_STATE},
    {"attitude_target", XGC_PORT_OUT_OPTIONAL, "xgc.attitude_target/1", XGC_QOS_STATE},
    {"vision_pose", XGC_PORT_IN_OPTIONAL, "xgc.pose/1", XGC_QOS_STATE},
    {"rigid_state_estimate", XGC_PORT_IN_OPTIONAL, "xgc.rigid_state_estimate/1", XGC_QOS_STATE},
    {"fcu_state", XGC_PORT_OUT_OPTIONAL, "xgc.fcu_state/1", XGC_QOS_STATE},
    {"local_pose", XGC_PORT_OUT_OPTIONAL, "xgc.pose/1", XGC_QOS_STATE},
    {"local_velocity", XGC_PORT_OUT_OPTIONAL, "xgc.twist/1", XGC_QOS_STATE},
    {"fcu_imu", XGC_PORT_OUT_OPTIONAL, "xgc.imu/1", XGC_QOS_STATE},
    {"battery", XGC_PORT_OUT_OPTIONAL, "xgc.battery/1", XGC_QOS_STATE},
    {"command", XGC_PORT_OUT_OPTIONAL, "xgc.command/1", XGC_QOS_EVENT},
    {"alg_setpoint", XGC_PORT_OUT_OPTIONAL, "xgc.position_target/1", XGC_QOS_CONTROL},
    {"setpoint", XGC_PORT_IN_OPTIONAL, "xgc.position_target/1", XGC_QOS_CONTROL},
    {"attitude_rate", XGC_PORT_IN_OPTIONAL, "xgc.body_rate_thrust/1", XGC_QOS_CONTROL},
    {"status", XGC_PORT_IN_OPTIONAL, "xgc.controller_status/1", XGC_QOS_STATE},
    {"fcu_request", XGC_PORT_IN_OPTIONAL, "xgc.fcu_request/1", XGC_QOS_EVENT},
    {"ref_analytic", XGC_PORT_OUT_OPTIONAL, "xgc.ref.analytic/1", XGC_QOS_EVENT},
    {"ref_sampled", XGC_PORT_OUT_OPTIONAL, "xgc.ref.sampled/1", XGC_QOS_EVENT},
    {"ref_reset", XGC_PORT_OUT_OPTIONAL, "xgc.ref.reset/1", XGC_QOS_EVENT},
    {"ref_status", XGC_PORT_IN_OPTIONAL, "xgc.ref.status/1", XGC_QOS_STATE},
    {"ref_active_analytic", XGC_PORT_IN_OPTIONAL, "xgc.ref.analytic/1", XGC_QOS_STATE},
    {"ref_active_sampled", XGC_PORT_IN_OPTIONAL, "xgc.ref.sampled/1", XGC_QOS_STATE},
    {"hover_thrust", XGC_PORT_OUT_OPTIONAL, "xgc.hover_thrust/1", XGC_QOS_STATE},
    {"controller_state", XGC_PORT_OUT_OPTIONAL, "xgc.controller_status/1", XGC_QOS_STATE},
    {"planar_pva", XGC_PORT_IN_OPTIONAL, "xgc.planar_pva/1", XGC_QOS_CONTROL},
    {"sim_pose", XGC_PORT_IN_OPTIONAL, "xgc.pose/1", XGC_QOS_STATE},
    {"sim_velocity", XGC_PORT_IN_OPTIONAL, "xgc.twist/1", XGC_QOS_STATE},
    {"sim_imu", XGC_PORT_IN_OPTIONAL, "xgc.imu/1", XGC_QOS_STATE},
    {"sim_fcu_state", XGC_PORT_IN_OPTIONAL, "xgc.fcu_state/1", XGC_QOS_STATE},
    {"cmd_vel", XGC_PORT_OUT_OPTIONAL, "xgc.twist/1", XGC_QOS_CONTROL},
    {"sim_fcu_request", XGC_PORT_OUT_OPTIONAL, "xgc.fcu_request/2", XGC_QOS_EVENT},
    {"attitude_target_full", XGC_PORT_OUT_OPTIONAL, "xgc.attitude_target/2", XGC_QOS_CONTROL},
    {"sim_attitude_target", XGC_PORT_IN_OPTIONAL, "xgc.attitude_target/2", XGC_QOS_STATE},
    {"sim_hover_thrust", XGC_PORT_IN_OPTIONAL, "xgc.hover_thrust/1", XGC_QOS_STATE},
    {"sim_fcu_result", XGC_PORT_IN_OPTIONAL, "xgc.fcu_result/1", XGC_QOS_EVENT},
    {"sim_extended_state", XGC_PORT_IN_OPTIONAL, "xgc.fcu_extended_state/1", XGC_QOS_STATE},
    {"sim_provider_request", XGC_PORT_OUT_OPTIONAL, "xgc.sim_provider_request/1", XGC_QOS_EVENT},
    {"sim_provider_result", XGC_PORT_IN_OPTIONAL, "xgc.sim_provider_result/1", XGC_QOS_EVENT},
};

const xgc_plugin_vtbl kVtbl = {create, configure, activate, step, deactivate, destroy, domain_state};

const xgc_plugin_descriptor kDescriptor = {
    XGC_RT_ABI_VERSION, kPortCount, "ros-io", "0.2.0", kPorts, &kVtbl,
};

}  // namespace

// ros_sim_edge.cpp includes this file for `RosIo` and brings its own entry.
#ifndef XGC_ROS_IO_NO_ENTRY
extern "C" __attribute__((visibility("default"))) const xgc_plugin_descriptor* xgc_rt_plugin_v1(void) {
  return &kDescriptor;
}
#endif
