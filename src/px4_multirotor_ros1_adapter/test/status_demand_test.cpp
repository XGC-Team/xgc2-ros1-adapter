#include <gtest/gtest.h>
#include <chrono>
#include <cmath>
#include <atomic>
#include <condition_variable>
#include <future>
#include <limits>
#include <mutex>
#include <thread>
#include <vector>
#include <ros/ros.h>
#include <zenoh.h>

#include <unistd.h>
#include "xgc_px4_multirotor_ros1_adapter/generated_contract.hpp"
#include "xgc_px4_multirotor_ros1_adapter/robot_runtime.hpp"
#include "xgc/semantic/common/v1/telemetry.pb.h"
#include "xgc/semantic/aerial/v1/diagnostic.pb.h"
#include "xgc2/vrpn_zenoh/v1/tracker.pb.h"

namespace xgc_px4_multirotor_ros1_adapter {
class ZenohPosePublisher {
public:
  ZenohPosePublisher(const std::string &endpoint, const std::string &key) {
    const std::string config =
        "{\"mode\":\"peer\",\"listen\":{\"endpoints\":[\"" + endpoint +
        "\"]},\"scouting\":{\"multicast\":{\"enabled\":false},"
        "\"gossip\":{\"enabled\":false}}}";
    if (zc_config_from_str(&config_, config.c_str()) < 0 ||
        z_open(&session_, z_move(config_), nullptr) < 0) {
      return;
    }
    session_open_ = true;
    if (z_view_keyexpr_from_str(&key_expression_, key.c_str()) < 0 ||
        z_declare_publisher(z_loan(session_), &publisher_,
                            z_loan(key_expression_), nullptr) < 0) {
      return;
    }
    publisher_open_ = true;
  }
  ~ZenohPosePublisher() {
    if (publisher_open_) z_drop(z_move(publisher_));
    if (session_open_) z_drop(z_move(session_));
  }
  bool ready() const { return publisher_open_; }
  void publish(const xgc2::vrpn_zenoh::v1::Pose &sample) {
    std::string payload;
    if (!sample.SerializeToString(&payload)) return;
    z_owned_bytes_t bytes;
    if (z_bytes_copy_from_buf(
            &bytes, reinterpret_cast<const std::uint8_t *>(payload.data()),
            payload.size()) < 0)
      return;
    z_publisher_put(z_loan(publisher_), z_move(bytes), nullptr);
  }
private:
  z_owned_config_t config_{};
  z_owned_session_t session_{};
  z_view_keyexpr_t key_expression_{};
  z_owned_publisher_t publisher_{};
  bool session_open_ = false, publisher_open_ = false;
};

class StatusDemandHarness {
public:
  explicit StatusDemandHarness(bool live_sources = false,
                               const std::string &canonical_pose = "/demand_sources/canonical_pose",
                               const std::string &vision_pose = "/demand_sources/vision",
                               const xgc2_ros1_robot_adapter::LocalizationProjectionConfig &localization = {},
                               const xgc2_ros1_robot_adapter::ZenohLocalizationConfig &zenoh = {},
                               double localization_delay_seconds = 0.0,
                               const VisionInjectionGateConfig &vision_injection_gate = {},
                               const std::string &robot_id = "r1",
                               const std::string &prefix = "/demand_sources") {
    ros::NodeHandle node;
    std::set<std::string> channels{"state.pose", "state.mocap.pose", "state.vision.pose", "state.velocity",
        "state.mocap.velocity", "state.mocap.acceleration", "state.mocap.speed", "state.localization.error",
        "state.imu", "state.power", "state.health", "state.flight", "state.controller", "setpoint.local",
        "setpoint.attitude", "diagnostic.fcu-link", "diagnostic.offboard-input", "diagnostic.stream-health"};
    NativeProfileConfig native;
    native.publish_vision = true;
    native.localization = localization;
    native.localization_delay_seconds = localization_delay_seconds;
    native.zenoh_localization = zenoh;
    native.vision_injection_gate = vision_injection_gate;
    if (live_sources) {
      const std::string base_prefix =
          prefix.empty() ? std::string("/demand_sources") : prefix;
      native.pose_endpoint = base_prefix + "/pose";
      native.mocap_endpoint = base_prefix + "/mocap";
      native.mocap_velocity_endpoint = base_prefix + "/mocap_velocity";
      native.mocap_acceleration_endpoint = base_prefix + "/mocap_acceleration";
      native.canonical_pose_endpoint = canonical_pose;
      native.canonical_velocity_endpoint = base_prefix + "/canonical_velocity";
      native.canonical_acceleration_endpoint = base_prefix + "/canonical_acceleration";
      native.vision_pose_endpoint = vision_pose;
      native.velocity_endpoint = base_prefix + "/velocity";
      native.imu_endpoint = base_prefix + "/imu";
      native.power_endpoint = base_prefix + "/power";
      native.controller_status_endpoint = base_prefix + "/controller";
      native.state_endpoint = base_prefix + "/state";
      native.extended_state_endpoint = base_prefix + "/extended";
      native.local_setpoint_endpoint = base_prefix + "/local_setpoint";
      native.attitude_setpoint_endpoint = base_prefix + "/attitude_setpoint";
      native.timesync_endpoint = base_prefix + "/timesync";
    }
    const std::string ns =
        (!robot_id.empty() && robot_id.front() == '/') ? robot_id : ("/" + robot_id);
    runtime_.reset(new RobotRuntime(std::move(node), robot_id, InstalledContract().primaryProfile().profile_id,
        ns, channels, channels, std::move(native), {},
        [this](xgc::robot::v1::RobotMessage message) {
          std::unique_lock<std::mutex> lock(mutex_);
          messages_.push_back(std::move(message));
          if (block_) {
            entered_ = true;
            changed_.notify_all();
            changed_.wait(lock, [this] { return released_; });
          }
        }));
    if (live_sources) {
      std::string error;
      if (!runtime_->install(&error))
        throw std::runtime_error(error);
    } else {
      runtime_->vision_pose_publisher_ = runtime_->node_handle_.advertise<geometry_msgs::PoseStamped>("status_test_vision", 1);
      std::lock_guard<std::mutex> lock(runtime_->mutex_);
      for (const auto &channel : channels)
        runtime_->ensureSourceLocked(channel, 5.0);
    }
  }

  ~StatusDemandHarness() {
    runtime_->Stop();
    runtime_.reset();
  }
  void pose(double x, const ros::Time &stamp = ros::Time(12, 3)) {
    geometry_msgs::PoseStamped::Ptr value(new geometry_msgs::PoseStamped);
    value->header.frame_id = "world";
    value->header.stamp = stamp;
    value->pose.position.x = x;
    value->pose.orientation.w = 1;
    runtime_->px4PoseCallback(value);
  }
  void imu() {
    sensor_msgs::Imu::Ptr value(new sensor_msgs::Imu);
    value->header.frame_id = "body";
    runtime_->imuCallback(value);
  }
  void state() {
    mavros_msgs::State::Ptr value(new mavros_msgs::State);
    value->connected = true;
    value->mode = "OFFBOARD";
    runtime_->mavrosStateCallback(value);
    mavros_msgs::ExtendedState::Ptr extended(new mavros_msgs::ExtendedState);
    runtime_->mavrosExtendedStateCallback(extended);
  }
  void mocap(double x, const ros::Time &stamp = ros::Time(12, 3)) {
    mocapPose(x, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, stamp);
  }
  void mocapPose(double x, double y, double z, double qx, double qy, double qz,
                 double qw, const ros::Time &stamp = ros::Time(12, 3)) {
    geometry_msgs::PoseStamped::Ptr value(new geometry_msgs::PoseStamped);
    value->header.frame_id = "world";
    value->header.stamp = stamp;
    value->pose.position.x = x;
    value->pose.position.y = y;
    value->pose.position.z = z;
    value->pose.orientation.x = qx;
    value->pose.orientation.y = qy;
    value->pose.orientation.z = qz;
    value->pose.orientation.w = qw;
    runtime_->mocapPoseCallback(value);
  }
  void localPose(double x, double y, double z, double qx, double qy, double qz,
                 double qw, const ros::Time &stamp = ros::Time(12, 3)) {
    geometry_msgs::PoseStamped::Ptr value(new geometry_msgs::PoseStamped);
    value->header.frame_id = "world";
    value->header.stamp = stamp;
    value->pose.position.x = x;
    value->pose.position.y = y;
    value->pose.position.z = z;
    value->pose.orientation.x = qx;
    value->pose.orientation.y = qy;
    value->pose.orientation.z = qz;
    value->pose.orientation.w = qw;
    runtime_->px4PoseCallback(value);
  }
  void publishZenohSample(double x, double y, double z, double qx, double qy,
                          double qz, double qw, std::int64_t report_time_nanos,
                          xgc::v1::ClockDomain clock_domain =
                              xgc::v1::CLOCK_DOMAIN_STEADY) {
    xgc2_ros1_robot_adapter::ZenohPoseSample sample;
    sample.position[0] = x;
    sample.position[1] = y;
    sample.position[2] = z;
    sample.orientation[0] = qx;
    sample.orientation[1] = qy;
    sample.orientation[2] = qz;
    sample.orientation[3] = qw;
    sample.report_time_nanos = report_time_nanos;
    sample.clock_domain = static_cast<std::uint8_t>(clock_domain);
    runtime_->zenohPoseCallback(sample);
  }
  void localSetpoint(bool valid) {
    mavros_msgs::PositionTarget::Ptr value(new mavros_msgs::PositionTarget);
    value->coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    value->position.x = valid ? 1 : std::numeric_limits<double>::quiet_NaN();
    runtime_->localSetpointCallback(value);
  }
  void seedAll() {
    pose(11);
    imu();
    state();
    geometry_msgs::PoseStamped::Ptr mocap(new geometry_msgs::PoseStamped);
    mocap->header.frame_id = "world";
    mocap->header.stamp = ros::Time(12, 3);
    mocap->pose.position.x = 8;
    mocap->pose.orientation.w = 1;
    runtime_->mocapPoseCallback(mocap);
    geometry_msgs::TwistStamped::Ptr velocity(new geometry_msgs::TwistStamped);
    velocity->header.frame_id = "world";
    velocity->twist.linear.x = 3;
    runtime_->mocapVelocityCallback(velocity);
    runtime_->px4VelocityCallback(velocity);
    geometry_msgs::AccelStamped::Ptr acceleration(new geometry_msgs::AccelStamped);
    acceleration->header.frame_id = "world";
    acceleration->accel.linear.x = 4;
    runtime_->mocapAccelerationCallback(acceleration);
    sensor_msgs::BatteryState::Ptr power(new sensor_msgs::BatteryState);
    power->percentage = 0.5;
    power->voltage = 24;
    runtime_->batteryCallback(power);
    std_msgs::String::Ptr controller(new std_msgs::String);
    controller->data = "active";
    runtime_->controllerStatusCallback(controller);
    mavros_msgs::PositionTarget::Ptr local(new mavros_msgs::PositionTarget);
    local->header.frame_id = "world";
    local->coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    runtime_->localSetpointCallback(local);
    mavros_msgs::AttitudeTarget::Ptr attitude(new mavros_msgs::AttitudeTarget);
    attitude->header.frame_id = "world";
    attitude->orientation.w = 1;
    runtime_->attitudeSetpointCallback(attitude);
    mavros_msgs::TimesyncStatus::Ptr sync(new mavros_msgs::TimesyncStatus);
    sync->remote_timestamp_ns = 42;
    runtime_->timesyncStatusCallback(sync);
  }
  void zenohPose(double x, std::int64_t report_ns,
                 const std::string &frame = "world") {
    xgc2_ros1_robot_adapter::ZenohPoseSample sample;
    sample.report_time_nanos = report_ns;
    sample.clock_domain = static_cast<std::int32_t>(xgc::v1::CLOCK_DOMAIN_SIMULATION);
    sample.position[0] = x;
    sample.orientation[3] = 1.0;
    runtime_->zenoh_localization_.frame_id = frame;
    runtime_->zenohPoseCallback(sample);
  }
  void zenohVelocity(double linear_x, double rotation_z, double interval,
                     std::int64_t report_ns) {
    xgc2_ros1_robot_adapter::ZenohVelocitySample sample;
    sample.report_time_nanos = report_ns;
    sample.clock_domain = static_cast<std::int32_t>(xgc::v1::CLOCK_DOMAIN_SIMULATION);
    sample.linear[0] = linear_x;
    sample.rotation[2] = rotation_z;
    sample.rotation[3] = 0.7071067811865476;
    sample.interval_seconds = interval;
    runtime_->zenohVelocityCallback(sample);
  }
  void zenohAcceleration(double linear_x, double rotation_z, double interval,
                         std::int64_t report_ns) {
    xgc2_ros1_robot_adapter::ZenohAccelerationSample sample;
    sample.report_time_nanos = report_ns;
    sample.clock_domain = static_cast<std::int32_t>(xgc::v1::CLOCK_DOMAIN_SIMULATION);
    sample.linear[0] = linear_x;
    sample.rotation[2] = rotation_z;
    sample.rotation[3] = 0.7071067811865476;
    sample.interval_seconds = interval;
    runtime_->zenohAccelerationCallback(sample);
  }
  void periodic() { runtime_->emitPeriodic(); }
  // Model a real callback recording its first source after Periodic captured
  // time, but before Periodic acquires the runtime's source lock.
  void periodicAcrossFirstSource() {
    std::promise<ros::WallTime> captured;
    auto captured_future = captured.get_future();
    std::unique_lock<std::mutex> lock(runtime_->mutex_);
    auto &source = runtime_->sources_.at("state.imu");
    const auto stale_after = source.stale_after_seconds;
    source = RobotRuntime::SourceTracker{};
    source.stale_after_seconds = stale_after;
    runtime_->last_output_.erase("diagnostic.stream-health");
    std::thread periodic([&] {
      const auto now = ros::WallTime::now();
      captured.set_value(now);
      runtime_->emitPeriodic();
    });
    const auto before = captured_future.get();
    auto observed = ros::WallTime::now();
    while (observed <= before) observed = ros::WallTime::now();
    runtime_->recordSourceLocked("state.imu", observed);
    lock.unlock();
    periodic.join();
  }
  void stop() { runtime_->Stop(); }
  void demand(std::set<std::string> channels) { runtime_->SetStatusDemand(channels); }
  std::pair<bool, std::string> health() { return runtime_->DescribeHealth(); }
  std::uint64_t encoded() {
    std::lock_guard<std::mutex> lock(runtime_->mutex_);
    std::uint64_t total = 0;
    for (const auto &entry : runtime_->sequences_) total += entry.second;
    return total;
  }
  std::uint64_t rawObserved(const std::string &channel) {
    std::lock_guard<std::mutex> lock(runtime_->mutex_);
    return runtime_->sources_.at(channel).last_seen.toNSec();
  }
  std::vector<xgc::robot::v1::RobotMessage> messages() {
    std::lock_guard<std::mutex> lock(mutex_);
    return messages_;
  }
  void block() { std::lock_guard<std::mutex> lock(mutex_); block_ = true; }
  bool waitEntered() {
    std::unique_lock<std::mutex> lock(mutex_);
    return changed_.wait_for(lock, std::chrono::seconds(2), [this] { return entered_; });
  }
  void release() {
    std::lock_guard<std::mutex> lock(mutex_);
    released_ = true;
    changed_.notify_all();
  }
private:
  std::shared_ptr<RobotRuntime> runtime_;
  std::mutex mutex_;
  std::condition_variable changed_;
  std::vector<xgc::robot::v1::RobotMessage> messages_;
  bool block_ = false, entered_ = false, released_ = false;
};

template <class Predicate>
bool waitFor(Predicate ready, double seconds = 3.0) {
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(seconds);
  while (std::chrono::steady_clock::now() < deadline) {
    if (ready()) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return ready();
}

TEST(StatusDemand, ZenohOpenReachesPx4CanonicalPoseAndVision) {
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::NodeHandle node;
  const std::uint16_t port =
      static_cast<std::uint16_t>(20000u + (getpid() % 20000) + 4u);
  const std::string endpoint = "tcp/127.0.0.1:" + std::to_string(port);
  xgc2_ros1_robot_adapter::ZenohLocalizationBinding binding;
  binding.endpoint = endpoint;
  binding.source = "gazebo";
  binding.tracker = "tracked_body";
  binding.sensor = 0;
  ZenohPosePublisher publisher(endpoint,
                               xgc2_ros1_robot_adapter::ZenohLocalizationKey(binding, "pose"));
  ASSERT_TRUE(publisher.ready());
  xgc2_ros1_robot_adapter::LocalizationProjectionConfig localization;
  localization.offset_x = 1.25;
  xgc2_ros1_robot_adapter::ZenohLocalizationConfig zenoh;
  zenoh.enabled = true;
  zenoh.endpoint = endpoint;
  zenoh.source = "gazebo";
  zenoh.tracker = "tracked_body";
  zenoh.sensor = 0;
  zenoh.frame_id = "world";
  StatusDemandHarness source(true, "/slot4/pose", "/slot4/mavros/vision_pose/pose",
                             localization, zenoh);
  std::atomic<unsigned> pose_received{0}, vision_received{0};
  auto pose = node.subscribe<geometry_msgs::PoseStamped>("/slot4/pose", 10,
      [&](const geometry_msgs::PoseStamped::ConstPtr &) { ++pose_received; });
  auto vision = node.subscribe<geometry_msgs::PoseStamped>("/slot4/mavros/vision_pose/pose", 10,
      [&](const geometry_msgs::PoseStamped::ConstPtr &) { ++vision_received; });
  ASSERT_TRUE(waitFor([&] {
    xgc2::vrpn_zenoh::v1::Pose sample;
    sample.mutable_report_time()->set_nanoseconds(123456789012345LL);
    sample.mutable_report_time()->set_clock_domain(xgc::v1::CLOCK_DOMAIN_SIMULATION);
    sample.mutable_position_m()->set_x(2.5);
    sample.mutable_orientation_xyzw()->set_w(1.0);
    publisher.publish(sample);
    return pose_received.load() > 0 && vision_received.load() > 0;
  }));
  EXPECT_GT(pose_received.load(), 0u);
  EXPECT_GT(vision_received.load(), 0u);
  source.demand({"state.mocap.pose"});
  ASSERT_TRUE(waitFor([&] { return !source.messages().empty(); }));
  bool found = false;
  for (const auto &item : source.messages()) {
    if (item.channel_id() != "state.mocap.pose") continue;
    EXPECT_EQ(123456789012345LL, item.message().source_time().nanoseconds());
    found = true;
  }
  EXPECT_TRUE(found);
  source.stop();
  spinner.stop();
}

TEST(StatusDemand, ZenohPosePublishesSlotPoseAndVisionWithSourceTime) {
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::NodeHandle node;
  xgc2_ros1_robot_adapter::LocalizationProjectionConfig localization;
  localization.offset_x = 1.25;
  StatusDemandHarness source(true, "/slot4/pose", "/slot4/mavros/vision_pose/pose",
                             localization);
  std::atomic<unsigned> pose_received{0}, vision_received{0};
  auto pose = node.subscribe<geometry_msgs::PoseStamped>("/slot4/pose", 10,
      [&](const geometry_msgs::PoseStamped::ConstPtr &) { ++pose_received; });
  auto vision = node.subscribe<geometry_msgs::PoseStamped>("/slot4/mavros/vision_pose/pose", 10,
      [&](const geometry_msgs::PoseStamped::ConstPtr &) { ++vision_received; });
  ASSERT_TRUE(waitFor([&] { return pose.getNumPublishers() > 0 && vision.getNumPublishers() > 0; }));
  source.demand({"state.mocap.pose"});
  source.zenohPose(2.5, 123456789012345LL);
  ASSERT_TRUE(waitFor([&] { return pose_received.load() > 0 && vision_received.load() > 0; }));

  source.demand({"state.mocap.pose", "state.vision.pose"});
  ASSERT_TRUE(waitFor([&] { return !source.messages().empty(); }));
  bool found_pose = false, found_vision = false;
  for (const auto &item : source.messages()) {
    if (item.channel_id() == "state.mocap.pose") {
      found_pose = true;
      EXPECT_EQ(123456789012345LL, item.message().source_time().nanoseconds());
      EXPECT_EQ(xgc::v1::CLOCK_DOMAIN_SIMULATION,
                item.message().source_time().clock_domain());
    } else if (item.channel_id() == "state.vision.pose") {
      found_vision = true;
      EXPECT_EQ(123456789012345LL, item.message().source_time().nanoseconds());
    }
  }
  EXPECT_TRUE(found_pose);
  EXPECT_TRUE(found_vision);
  source.stop();
  spinner.stop();
}

TEST(StatusDemand, JitteredUpstreamPoseReachesVisionOnceWithWorldOffset) {
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::NodeHandle node;
  xgc2_ros1_robot_adapter::LocalizationProjectionConfig localization;
  localization.offset_x = 1.25;
  localization.offset_y = -2.0;
  localization.offset_z = 0.5;
  StatusDemandHarness source(true, "/jitter/pose", "/jitter/vision", localization);
  std::mutex received_mutex;
  std::vector<geometry_msgs::PoseStamped> received;
  auto vision = node.subscribe<geometry_msgs::PoseStamped>("/jitter/vision", 100,
      [&](const geometry_msgs::PoseStamped::ConstPtr &message) {
        std::lock_guard<std::mutex> lock(received_mutex);
        received.push_back(*message);
      });
  ASSERT_TRUE(waitFor([&] { return vision.getNumPublishers() > 0; }));
  const auto start = std::chrono::steady_clock::now();
  // Average 30.3 Hz, alternating 18/48 ms arrivals as seen after transport
  // jitter. A second 30 Hz gate must not discard the shorter intervals.
  for (int sample = 0; sample < 40; ++sample) {
    const auto millis = (sample / 2) * 66 + (sample % 2) * 18;
    std::this_thread::sleep_until(start + std::chrono::milliseconds(millis));
    source.zenohPose(sample, 1000000000LL + millis * 1000000LL);
  }
  ASSERT_TRUE(waitFor([&] {
    std::lock_guard<std::mutex> lock(received_mutex);
    return received.size() >= 40;
  }));
  {
    std::lock_guard<std::mutex> lock(received_mutex);
    ASSERT_EQ(40u, received.size());
    for (std::size_t index = 0; index < received.size(); ++index) {
      EXPECT_DOUBLE_EQ(index + 1.25, received[index].pose.position.x);
      EXPECT_DOUBLE_EQ(-2.0, received[index].pose.position.y);
      EXPECT_DOUBLE_EQ(0.5, received[index].pose.position.z);
      EXPECT_DOUBLE_EQ(1.0, received[index].pose.orientation.w);
    }
  }
  source.stop();
  spinner.stop();
}

TEST(StatusDemand, ZenohVelocityAndAccelerationFeedCanonicalOutputs) {
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::NodeHandle node;
  StatusDemandHarness source(true);
  std::atomic<unsigned> twist_received{0}, accel_received{0};
  geometry_msgs::TwistStamped twist;
  geometry_msgs::AccelStamped accel;
  auto twist_sub = node.subscribe<geometry_msgs::TwistStamped>("/demand_sources/canonical_velocity", 10,
      [&](const geometry_msgs::TwistStamped::ConstPtr &message) {
        twist = *message; ++twist_received;
      });
  auto accel_sub = node.subscribe<geometry_msgs::AccelStamped>("/demand_sources/canonical_acceleration", 10,
      [&](const geometry_msgs::AccelStamped::ConstPtr &message) {
        accel = *message; ++accel_received;
      });
  ASSERT_TRUE(waitFor([&] { return twist_sub.getNumPublishers() > 0 && accel_sub.getNumPublishers() > 0; }));
  source.demand({"state.mocap.velocity", "state.mocap.acceleration"});
  source.zenohVelocity(3.0, 0.7071067811865476, 1.0, 42);
  source.zenohAcceleration(4.0, 0.7071067811865476, 1.0, 43);
  ASSERT_TRUE(waitFor([&] { return twist_received.load() > 0 && accel_received.load() > 0; }));
  EXPECT_NEAR(3.0, twist.twist.linear.x, 1e-12);
  EXPECT_NEAR(1.5707963267948966, twist.twist.angular.z, 1e-12);
  EXPECT_NEAR(4.0, accel.accel.linear.x, 1e-12);
  EXPECT_NEAR(1.5707963267948966, accel.accel.angular.z, 1e-12);
  source.stop();
  spinner.stop();
}

TEST(StatusDemand, RealRosInputsFollowUnionAndKeepIndependentLocalization) {
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::NodeHandle node;
  StatusDemandHarness source(true);
  auto imu = node.advertise<sensor_msgs::Imu>("/demand_sources/imu", 10, false);
  auto velocity = node.advertise<geometry_msgs::TwistStamped>("/demand_sources/velocity", 10, false);
  auto local = node.advertise<mavros_msgs::PositionTarget>("/demand_sources/local_setpoint", 10, false);
  auto attitude = node.advertise<mavros_msgs::AttitudeTarget>("/demand_sources/attitude_setpoint", 10, false);
  auto timesync = node.advertise<mavros_msgs::TimesyncStatus>("/demand_sources/timesync", 10, false);
  auto state = node.advertise<mavros_msgs::State>("/demand_sources/state", 10, false);
  auto mocap = node.advertise<geometry_msgs::PoseStamped>("/demand_sources/mocap", 10, false);
  std::atomic<unsigned> forwarded{0};
  auto canonical = node.subscribe<geometry_msgs::PoseStamped>("/demand_sources/canonical_pose", 10,
      [&](const geometry_msgs::PoseStamped::ConstPtr &) { ++forwarded; });

  ASSERT_TRUE(waitFor([&] { return state.getNumSubscribers() && mocap.getNumSubscribers() &&
                                  canonical.getNumPublishers(); }));
  EXPECT_EQ(0u, imu.getNumSubscribers());
  EXPECT_EQ(0u, velocity.getNumSubscribers());
  EXPECT_EQ(0u, local.getNumSubscribers());
  EXPECT_EQ(0u, timesync.getNumSubscribers());
  mavros_msgs::State connected;
  connected.connected = true;
  connected.mode = "OFFBOARD";
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "world";
  pose.pose.orientation.w = 1;
  ASSERT_TRUE(waitFor([&] {
    state.publish(connected);
    mocap.publish(pose);
    return source.health().first && forwarded.load() > 0;
  }));
  EXPECT_TRUE(source.messages().empty());

  source.demand({"state.imu"});
  ASSERT_TRUE(waitFor([&] { return imu.getNumSubscribers() == 1; }));
  EXPECT_EQ(0u, velocity.getNumSubscribers());
  sensor_msgs::Imu sample;
  sample.header.frame_id = "old-generation";
  ASSERT_TRUE(waitFor([&] { imu.publish(sample); return !source.messages().empty(); }));
  source.demand({"state.velocity"});
  ASSERT_TRUE(waitFor([&] { return !imu.getNumSubscribers() && velocity.getNumSubscribers() == 1; }));
  const auto before = source.messages().size();
  sample.header.frame_id = "while-unsubscribed";
  imu.publish(sample);
  source.demand({"state.imu"});
  ASSERT_TRUE(waitFor([&] { return imu.getNumSubscribers() == 1 && !velocity.getNumSubscribers(); }));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(before, source.messages().size()); // no retired source replay
  sample.header.frame_id = "new-generation";
  ASSERT_TRUE(waitFor([&] { imu.publish(sample); return source.messages().size() > before; }));
  xgc::semantic::common::v1::ImuEstimate observed;
  ASSERT_TRUE(observed.ParseFromString(source.messages().back().message().payload().value()));
  EXPECT_EQ("new-generation", observed.frame_id());

  // A derived diagnostic owns the two source subscriptions even when their
  // raw channels are not selected. A stream-health consumer needs all three.
  source.demand({"diagnostic.offboard-input"});
  ASSERT_TRUE(waitFor([&] { return !imu.getNumSubscribers() && local.getNumSubscribers() == 1 &&
                                  attitude.getNumSubscribers() == 1; }));
  EXPECT_EQ(0u, timesync.getNumSubscribers());
  source.demand({"diagnostic.stream-health"});
  ASSERT_TRUE(waitFor([&] { return imu.getNumSubscribers() == 1 && velocity.getNumSubscribers() == 1 &&
                                  local.getNumSubscribers() == 1 && attitude.getNumSubscribers() == 1 &&
                                  timesync.getNumSubscribers() == 1; }));
  source.demand({});
  ASSERT_TRUE(waitFor([&] { return !imu.getNumSubscribers() && !velocity.getNumSubscribers() &&
                                  !local.getNumSubscribers() && !attitude.getNumSubscribers() &&
                                  !timesync.getNumSubscribers(); }));
  EXPECT_EQ(1u, state.getNumSubscribers());
  EXPECT_EQ(1u, mocap.getNumSubscribers());
  source.stop();
  spinner.stop();
}

TEST(StatusDemand, RealRosRetirementJoinsAnInFlightProducer) {
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::NodeHandle node;
  StatusDemandHarness source(true);
  auto imu = node.advertise<sensor_msgs::Imu>("/demand_sources/imu", 10, false);
  source.demand({"state.imu"});
  ASSERT_TRUE(waitFor([&] { return imu.getNumSubscribers() == 1; }));
  source.block();
  sensor_msgs::Imu value;
  value.header.frame_id = "body";
  ASSERT_TRUE(waitFor([&] { imu.publish(value); return source.waitEntered(); }));
  auto retirement = std::async(std::launch::async, [&] { source.demand({}); });
  EXPECT_EQ(std::future_status::timeout, retirement.wait_for(std::chrono::milliseconds(30)));
  source.release();
  ASSERT_EQ(std::future_status::ready, retirement.wait_for(std::chrono::seconds(3)));
  retirement.get();
  EXPECT_TRUE(waitFor([&] { return imu.getNumSubscribers() == 0; }));
  const auto count = source.messages().size();
  imu.publish(value);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(count, source.messages().size());
  source.stop();
  spinner.stop();
}

TEST(StatusDemand, RetiringStatusLeavesAnotherLocalRosConsumerSubscribed) {
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::NodeHandle node;
  StatusDemandHarness source(true);
  auto imu = node.advertise<sensor_msgs::Imu>("/demand_sources/imu", 10, false);
  std::atomic<unsigned> local_received{0};
  auto local = node.subscribe<sensor_msgs::Imu>("/demand_sources/imu", 10,
      [&](const sensor_msgs::Imu::ConstPtr &) { ++local_received; });
  ASSERT_TRUE(waitFor([&] { return imu.getNumSubscribers() > 0; }));
  sensor_msgs::Imu value;
  value.header.frame_id = "body";
  ASSERT_TRUE(waitFor([&] { imu.publish(value); return local_received.load() > 0; }));
  EXPECT_TRUE(source.messages().empty());
  source.demand({"state.imu"});
  ASSERT_TRUE(waitFor([&] { imu.publish(value); return !source.messages().empty(); }));
  source.demand({});
  const auto before = local_received.load();
  ASSERT_TRUE(waitFor([&] { imu.publish(value); return local_received.load() > before; }));
  EXPECT_TRUE(imu.getNumSubscribers() > 0);
  source.stop();
  spinner.stop();
}

TEST(StatusDemand, StopAndRetirementBothJoinARealRosCallback) {
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::NodeHandle node;
  StatusDemandHarness source(true);
  auto imu = node.advertise<sensor_msgs::Imu>("/demand_sources/imu", 10, false);
  source.demand({"state.imu"});
  ASSERT_TRUE(waitFor([&] { return imu.getNumSubscribers() == 1; }));
  source.block();
  sensor_msgs::Imu value;
  value.header.frame_id = "body";
  ASSERT_TRUE(waitFor([&] { imu.publish(value); return source.waitEntered(); }));
  auto retirement = std::async(std::launch::async, [&] { source.demand({}); });
  EXPECT_EQ(std::future_status::timeout, retirement.wait_for(std::chrono::milliseconds(30)));
  auto stopping = std::async(std::launch::async, [&] { source.stop(); });
  EXPECT_EQ(std::future_status::timeout, stopping.wait_for(std::chrono::milliseconds(30)));
  source.release();
  ASSERT_EQ(std::future_status::ready, retirement.wait_for(std::chrono::seconds(3)));
  retirement.get();
  ASSERT_EQ(std::future_status::ready, stopping.wait_for(std::chrono::seconds(3)));
  stopping.get();
  EXPECT_TRUE(waitFor([&] { return imu.getNumSubscribers() == 0; }));
  spinner.stop();
}

TEST(StatusDemand, PeriodicHealthSamplesTimeAfterTheSourceLock) {
  StatusDemandHarness source;
  source.demand({"state.imu", "diagnostic.stream-health"});
  source.periodicAcrossFirstSource();
  const auto frames = source.messages();
  bool found = false;
  for (auto it = frames.rbegin(); it != frames.rend(); ++it) {
    if (it->channel_id() != "diagnostic.stream-health") continue;
    xgc::semantic::common::v1::StreamHealthReport report;
    ASSERT_TRUE(report.ParseFromString(it->message().payload().value()));
    for (const auto &channel : report.channels()) {
      if (channel.channel_id() != "state.imu") continue;
      found = true;
      EXPECT_FALSE(channel.stale());
      EXPECT_GT(channel.source_rate_hz(), 0.0);
    }
    break;
  }
  EXPECT_TRUE(found);
}

TEST(StatusDemand, NoViewFreshSelectedSnapshotAndReopen) {
  StatusDemandHarness source;
  source.pose(1);
  source.pose(2);
  source.imu();
  source.state();
  source.periodic();
  ASSERT_TRUE(source.messages().empty());
  EXPECT_EQ(0u, source.encoded());
  const auto observed = source.rawObserved("state.pose");
  const auto health = source.health();
  EXPECT_TRUE(health.first);
  EXPECT_FALSE(health.second.empty());
  EXPECT_TRUE(source.messages().empty());
  source.demand({"state.pose"});
  auto frames = source.messages();
  ASSERT_EQ(1u, frames.size());
  xgc::semantic::common::v1::PoseEstimate pose;
  ASSERT_TRUE(pose.ParseFromString(frames[0].message().payload().value()));
  EXPECT_DOUBLE_EQ(2, pose.position().x());
  EXPECT_EQ(observed, static_cast<std::uint64_t>(frames[0].message().observed_unix_nanos()));
  source.imu();
  source.periodic();
  source.demand({"state.pose"});
  EXPECT_EQ(1u, source.messages().size());
  source.demand({});
  source.pose(3);
  source.imu();
  EXPECT_EQ(1u, source.messages().size());
  const auto latest_observed = source.rawObserved("state.pose");
  source.demand({"state.pose"});
  frames = source.messages();
  ASSERT_EQ(2u, frames.size());
  ASSERT_TRUE(pose.ParseFromString(frames[1].message().payload().value()));
  EXPECT_DOUBLE_EQ(3, pose.position().x());
  EXPECT_EQ(latest_observed, static_cast<std::uint64_t>(frames[1].message().observed_unix_nanos()));
  source.demand({});
}

TEST(StatusDemand, LastReleaseJoinsTheActualEmitter) {
  StatusDemandHarness source;
  source.demand({"state.pose"});
  source.block();
  std::thread producer([&] { source.pose(4); });
  const bool entered = source.waitEntered();
  if (!entered) {
    source.release();
    producer.join();
    FAIL() << "the selected source did not reach its emitter";
  }
  auto retirement = std::async(std::launch::async, [&] { source.demand({}); });
  EXPECT_EQ(std::future_status::timeout, retirement.wait_for(std::chrono::milliseconds(30)));
  source.release();
  producer.join();
  EXPECT_EQ(std::future_status::ready, retirement.wait_for(std::chrono::seconds(2)));
  retirement.get();
  const auto before = source.messages().size();
  source.pose(5);
  EXPECT_EQ(before, source.messages().size());
}

TEST(StatusDemand, EveryEnabledPx4ChannelSnapshotsOnlyItsOwnRawSource) {
  StatusDemandHarness source;
  source.seedAll();
  ASSERT_TRUE(source.messages().empty());
  EXPECT_EQ(0u, source.encoded());
  for (const std::string channel : {"state.pose", "state.mocap.pose", "state.vision.pose", "state.velocity",
                                    "state.mocap.velocity", "state.mocap.acceleration", "state.mocap.speed",
                                    "state.localization.error", "state.imu", "state.power", "state.health",
                                    "state.flight", "state.controller", "setpoint.local", "setpoint.attitude",
                                    "diagnostic.fcu-link", "diagnostic.offboard-input", "diagnostic.stream-health"}) {
    SCOPED_TRACE(channel);
    const auto before = source.messages().size();
    source.demand({channel});
    const auto frames = source.messages();
    ASSERT_EQ(before + 1, frames.size());
    EXPECT_EQ(channel, frames.back().channel_id());
    source.demand({});
  }
}

TEST(StatusDemand, LocalizationErrorLiveAndSnapshotUseTheSameTimedPair) {
  StatusDemandHarness source(false, "/unused_pose", "/unused_vision", {}, {}, 0.05);
  source.pose(0.0, ros::Time(10, 0));
  source.pose(0.1, ros::Time(10, 100000000));
  source.mocap(0.05, ros::Time(10, 100000000));
  ASSERT_TRUE(source.messages().empty());
  source.demand({"state.localization.error"});
  auto frames = source.messages();
  ASSERT_EQ(1u, frames.size());
  xgc::semantic::common::v1::DistanceEstimate error;
  ASSERT_TRUE(error.ParseFromString(frames.back().message().payload().value()));
  EXPECT_NEAR(0.0, error.meters(), 1e-12);
  EXPECT_EQ(10050000000LL, frames.back().message().source_time().nanoseconds());
  source.pose(0.2, ros::Time(10, 200000000));
  EXPECT_EQ(1u, source.messages().size());
  // A later reference callback must not recalculate the old mocap against now.
  source.demand({});
  source.demand({"state.localization.error"});
  frames = source.messages();
  ASSERT_EQ(2u, frames.size());
  ASSERT_TRUE(error.ParseFromString(frames.back().message().payload().value()));
  EXPECT_NEAR(0.0, error.meters(), 1e-12);
  EXPECT_EQ(10050000000LL, frames.back().message().source_time().nanoseconds());
}

TEST(StatusDemand, InvalidLatestMocapDoesNotReplayAnOlderValidSample) {
  StatusDemandHarness source;
  source.mocap(1);
  source.mocap(std::numeric_limits<double>::quiet_NaN());
  source.demand({"state.mocap.pose"});
  EXPECT_TRUE(source.messages().empty());
  source.demand({});
}

TEST(StatusDemand, InvalidLatestPositionDoesNotProduceAnOldLocalizationError) {
  StatusDemandHarness source;
  source.seedAll();
  source.mocap(std::numeric_limits<double>::quiet_NaN());
  source.demand({"state.localization.error"});
  EXPECT_TRUE(source.messages().empty());
  source.pose(11);
  source.mocap(3);
  ASSERT_EQ(1u, source.messages().size());
  source.demand({});
  source.pose(std::numeric_limits<double>::quiet_NaN());
  source.demand({"state.localization.error"});
  EXPECT_EQ(1u, source.messages().size());
  source.demand({});
}

TEST(StatusDemand, OffboardDiagnosticValidatesTheLatestRawSetpointOnDemand) {
  StatusDemandHarness source;
  source.state();
  source.localSetpoint(false);
  EXPECT_EQ(0u, source.encoded());
  source.demand({"diagnostic.offboard-input"});
  auto frames = source.messages();
  ASSERT_EQ(1u, frames.size());
  xgc::semantic::aerial::v1::OffboardInputStatus status;
  ASSERT_TRUE(status.ParseFromString(frames.back().message().payload().value()));
  bool invalid = false;
  for (const auto &blocker : status.blockers())
    invalid = invalid || blocker.code() == "offboard.setpoint.invalid";
  EXPECT_TRUE(invalid);
  source.demand({});
  source.localSetpoint(true);
  source.demand({"diagnostic.offboard-input"});
  frames = source.messages();
  ASSERT_EQ(2u, frames.size());
  ASSERT_TRUE(status.ParseFromString(frames.back().message().payload().value()));
  for (const auto &blocker : status.blockers())
    EXPECT_NE("offboard.setpoint.invalid", blocker.code());
  source.demand({});
}

TEST(StatusDemand, FrozenAdvancingStampAndRecoveryJumpReportHealthWithoutGatingVision) {
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::NodeHandle node;
  xgc2_ros1_robot_adapter::LocalizationProjectionConfig localization;
  localization.has_extrinsic = true;
  localization.marker_from_body_translation = {{0.05, 0.0, 0.0}};
  localization.marker_from_body_rotation = {{0.0, 0.0, 0.0, 1.0}};
  localization.offset_x = 1.0;
  localization.offset_y = -0.5;
  localization.offset_z = 0.25;
  StatusDemandHarness source(true, "/f10_recovery/pose", "/f10_recovery/vision",
                             localization, {}, 0.05);
  std::mutex received_mutex;
  std::vector<geometry_msgs::PoseStamped> received;
  auto vision = node.subscribe<geometry_msgs::PoseStamped>(
      "/f10_recovery/vision", 32,
      [&](const geometry_msgs::PoseStamped::ConstPtr &message) {
        std::lock_guard<std::mutex> lock(received_mutex);
        received.push_back(*message);
      });
  ASSERT_TRUE(waitFor([&] { return vision.getNumPublishers() > 0; }));

  auto has_fault = [](const xgc::semantic::common::v1::VehicleHealth &health,
                      const std::string &code) {
    for (const auto &fault : health.faults()) {
      if (fault.code() == code)
        return true;
    }
    return false;
  };

  const double expected_x = 3.05;
  const double expected_y = 0.5;
  const double expected_z = 1.75;

  // Replay "advancing stamp with identical pose" (new stamp, frozen P+Q).
  source.localPose(expected_x, expected_y, expected_z, 0.0, 0.0, 0.0, 1.0,
                   ros::Time(20, 0));
  for (int index = 0; index < 6; ++index) {
    const ros::Time mocap_stamp = ros::Time(20.05 + static_cast<double>(index) * 0.033);
    source.mocapPose(2.0, 1.0, 1.5, 0.0, 0.0, 0.0, 1.0, mocap_stamp);
  }
  ASSERT_TRUE(waitFor([&] {
    std::lock_guard<std::mutex> lock(received_mutex);
    return received.size() >= 6u;
  }));
  {
    std::lock_guard<std::mutex> lock(received_mutex);
    ASSERT_EQ(6u, received.size());
    for (const auto &pose : received) {
      EXPECT_DOUBLE_EQ(expected_x, pose.pose.position.x);
      EXPECT_DOUBLE_EQ(expected_y, pose.pose.position.y);
      EXPECT_DOUBLE_EQ(expected_z, pose.pose.position.z);
      EXPECT_DOUBLE_EQ(1.0, pose.pose.orientation.w);
    }
  }

  std::size_t before = source.messages().size();
  source.demand({"state.health"});
  auto frames = source.messages();
  ASSERT_GT(frames.size(), before);
  EXPECT_EQ("state.health", frames.back().channel_id());
  xgc::semantic::common::v1::VehicleHealth health;
  ASSERT_TRUE(health.ParseFromString(frames.back().message().payload().value()));
  using Health = xgc::semantic::common::v1::PositioningHealth;
  EXPECT_EQ(Health::POSITIONING_STATE_FROZEN, health.positioning().state());
  EXPECT_EQ(Health::POSITIONING_REASON_REPEAT_FRAME_WINDOW_FROZEN,
            health.positioning().reason());
  EXPECT_DOUBLE_EQ(0.0, health.positioning().window_spread_m());
  EXPECT_EQ(5u, health.positioning().sample_count());
  EXPECT_TRUE(has_fault(health, "mocap.position.frozen"));

  source.demand({});
  before = source.messages().size();
  source.demand({"state.vision.pose"});
  frames = source.messages();
  ASSERT_GT(frames.size(), before);
  xgc::semantic::common::v1::PoseEstimate vision_pose;
  ASSERT_TRUE(vision_pose.ParseFromString(frames.back().message().payload().value()));
  EXPECT_DOUBLE_EQ(expected_x, vision_pose.position().x());
  EXPECT_DOUBLE_EQ(expected_y, vision_pose.position().y());
  EXPECT_DOUBLE_EQ(expected_z, vision_pose.position().z());

  // Replay onsite 4.443 m recovery jump: injection continues without fallback
  // and health immediately switches to ACTIVE with exact spread and position error.
  source.demand({});
  source.localPose(expected_x, expected_y, expected_z, 0.0, 0.0, 0.0, 1.0,
                   ros::Time(20.35));
  source.mocapPose(2.0 + 4.443, 1.0, 1.5, 0.0, 0.0, 0.0, 1.0, ros::Time(20.40));
  ASSERT_TRUE(waitFor([&] {
    std::lock_guard<std::mutex> lock(received_mutex);
    return received.size() >= 7u;
  }));
  {
    std::lock_guard<std::mutex> lock(received_mutex);
    ASSERT_EQ(7u, received.size());
    EXPECT_NEAR(expected_x + 4.443, received.back().pose.position.x, 1e-12);
    EXPECT_DOUBLE_EQ(expected_y, received.back().pose.position.y);
    EXPECT_DOUBLE_EQ(expected_z, received.back().pose.position.z);
  }

  before = source.messages().size();
  source.demand({"state.health"});
  frames = source.messages();
  ASSERT_GT(frames.size(), before);
  ASSERT_TRUE(health.ParseFromString(frames.back().message().payload().value()));
  EXPECT_EQ(Health::POSITIONING_STATE_ACTIVE, health.positioning().state());
  EXPECT_EQ(Health::POSITIONING_REASON_WINDOW_VARIATION_OBSERVED,
            health.positioning().reason());
  EXPECT_NEAR(std::sqrt(5.0) * 4.443, health.positioning().window_spread_m(),
              1e-12);
  EXPECT_FALSE(has_fault(health, "mocap.position.frozen"));
  EXPECT_FALSE(has_fault(health, "mocap.position.timeout"));

  source.demand({});
  before = source.messages().size();
  source.demand({"state.localization.error"});
  frames = source.messages();
  ASSERT_GT(frames.size(), before);
  xgc::semantic::common::v1::DistanceEstimate error;
  ASSERT_TRUE(error.ParseFromString(frames.back().message().payload().value()));
  EXPECT_NEAR(4.443, error.meters(), 1e-9);

  // Exceeding the 100 ms XGC1 freshness boundary transitions state.health to TIMED_OUT.
  std::this_thread::sleep_for(std::chrono::milliseconds(115));
  source.demand({});
  before = source.messages().size();
  source.demand({"state.health"});
  frames = source.messages();
  ASSERT_GT(frames.size(), before);
  ASSERT_TRUE(health.ParseFromString(frames.back().message().payload().value()));
  EXPECT_EQ(Health::POSITIONING_STATE_TIMED_OUT, health.positioning().state());
  EXPECT_EQ(Health::POSITIONING_REASON_VRPN_TIMEOUT, health.positioning().reason());
  EXPECT_TRUE(has_fault(health, "mocap.position.timeout"));
  source.stop();
}

TEST(StatusDemand, AttitudeFlipInMocapKeepsSmallPositionErrorAndForwardsFlippedQuaternion) {
  constexpr double kPi = 3.14159265358979323846;
  StatusDemandHarness source(false, "/unused_pose", "/unused_vision", {}, {}, 0.05);
  source.localPose(1.0, -2.0, 0.5, 0.0, 0.0, 0.0, 1.0, ros::Time(30, 0));
  source.mocapPose(1.0, -2.0, 0.5, 0.0, 0.0, 0.0, 1.0, ros::Time(30.05));

  const double flip_angles[] = {179.0, 172.0, 178.0, 179.0, 172.0, 178.0};
  for (std::size_t index = 0; index < 6; ++index) {
    const double t = 30.1 + static_cast<double>(index) * 0.1;
    const double half = (flip_angles[index] * kPi / 180.0) * 0.5;
    source.localPose(1.0, -2.0, 0.5, 0.0, 0.0, 0.0, 1.0, ros::Time(t - 0.05));
    source.mocapPose(1.0, -2.0, 0.5, 0.0, 0.0, std::sin(half), std::cos(half),
                     ros::Time(t));

    source.demand({});
    std::size_t before = source.messages().size();
    source.demand({"state.localization.error"});
    auto frames = source.messages();
    ASSERT_GT(frames.size(), before);
    xgc::semantic::common::v1::DistanceEstimate error;
    ASSERT_TRUE(error.ParseFromString(frames.back().message().payload().value()));
    EXPECT_DOUBLE_EQ(0.0, error.meters());

    source.demand({});
    before = source.messages().size();
    source.demand({"state.vision.pose"});
    frames = source.messages();
    ASSERT_GT(frames.size(), before);
    xgc::semantic::common::v1::PoseEstimate vision;
    ASSERT_TRUE(vision.ParseFromString(frames.back().message().payload().value()));
    EXPECT_NEAR(std::sin(half), vision.orientation().z(), 1e-12);
    EXPECT_NEAR(std::cos(half), vision.orientation().w(), 1e-12);
  }

  source.demand({});
  const std::size_t before = source.messages().size();
  source.demand({"state.health"});
  const auto frames = source.messages();
  ASSERT_GT(frames.size(), before);
  xgc::semantic::common::v1::VehicleHealth health;
  ASSERT_TRUE(health.ParseFromString(frames.back().message().payload().value()));
  EXPECT_EQ(xgc::semantic::common::v1::PositioningHealth::POSITIONING_STATE_FROZEN,
            health.positioning().state());
  EXPECT_DOUBLE_EQ(0.0, health.positioning().window_spread_m());
}

TEST(StatusDemand, VisionInjectionGateEnforcesSixFieldRegimesWithoutFabrication) {
  constexpr double kPi = 3.14159265358979323846;
  VisionInjectionGateConfig gate_config;
  gate_config.enabled = true;
  StatusDemandHarness source(true, "/demand_sources_gate/canonical",
                             "/demand_sources_gate/vision", {}, {}, 0.05,
                             gate_config);

  ros::NodeHandle nh;
  std::mutex ros_mutex;
  std::vector<geometry_msgs::PoseStamped> canonical_poses;
  std::vector<geometry_msgs::PoseStamped> vision_poses;
  ros::Subscriber canonical_sub = nh.subscribe<geometry_msgs::PoseStamped>(
      "/demand_sources_gate/canonical", 256,
      [&](const geometry_msgs::PoseStamped::ConstPtr &msg) {
        std::lock_guard<std::mutex> lock(ros_mutex);
        canonical_poses.push_back(*msg);
      });
  ros::Subscriber vision_sub = nh.subscribe<geometry_msgs::PoseStamped>(
      "/demand_sources_gate/vision", 256,
      [&](const geometry_msgs::PoseStamped::ConstPtr &msg) {
        std::lock_guard<std::mutex> lock(ros_mutex);
        vision_poses.push_back(*msg);
      });

  const auto connect_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < connect_deadline) {
    ros::spinOnce();
    if (canonical_sub.getNumPublishers() > 0 &&
        vision_sub.getNumPublishers() > 0) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_GT(canonical_sub.getNumPublishers(), 0u);
  ASSERT_GT(vision_sub.getNumPublishers(), 0u);

  auto wait_ros = [&](std::size_t expected_canonical,
                      std::size_t expected_vision) {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < deadline) {
      ros::spinOnce();
      {
        std::lock_guard<std::mutex> lock(ros_mutex);
        if (canonical_poses.size() >= expected_canonical &&
            vision_poses.size() >= expected_vision) {
          break;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    // Give any unexpected extra vision messages a chance to arrive if broken.
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    ros::spinOnce();
    std::lock_guard<std::mutex> lock(ros_mutex);
    EXPECT_EQ(expected_canonical, canonical_poses.size());
    EXPECT_EQ(expected_vision, vision_poses.size());
  };

  auto fetch_health = [&]() {
    source.demand({});
    const std::size_t before = source.messages().size();
    source.demand({"state.health"});
    const auto frames = source.messages();
    xgc::semantic::common::v1::VehicleHealth health;
    for (std::size_t i = frames.size(); i > before; --i) {
      if (frames[i - 1].channel_id() == "state.health" &&
          health.ParseFromString(frames[i - 1].message().payload().value())) {
        return health;
      }
    }
    ADD_FAILURE() << "missing state.health response";
    return health;
  };

  auto has_fault = [](const xgc::semantic::common::v1::VehicleHealth &health,
                      const std::string &code) {
    for (const auto &fault : health.faults()) {
      if (fault.code() == code)
        return true;
    }
    return false;
  };

  auto fetch_latest_error = [&]() {
    source.demand({});
    const std::size_t before = source.messages().size();
    source.demand({"state.localization.error"});
    const auto frames = source.messages();
    for (std::size_t i = frames.size(); i > before; --i) {
      if (frames[i - 1].channel_id() == "state.localization.error") {
        xgc::semantic::common::v1::DistanceEstimate dist;
        if (dist.ParseFromString(frames[i - 1].message().payload().value())) {
          return dist.meters();
        }
      }
    }
    ADD_FAILURE() << "missing state.localization.error response";
    return -1.0;
  };

  double ros_t = 10.0;
  std::size_t expected_canonical = 0;
  std::size_t expected_vision = 0;

  // Regime 1: 20 pure-static frames (no +/-0.0005 jitter workaround) emit
  // mocap.position.frozen with zero injection_held and publish 20/20 vision.
  source.state();
  for (int i = 0; i < 20; ++i) {
    ros_t += 0.02;
    source.localPose(0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, ros::Time(ros_t - 0.05));
    source.mocapPose(0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, ros::Time(ros_t));
  }
  expected_canonical += 20;
  expected_vision += 20;
  wait_ros(expected_canonical, expected_vision);
  {
    const auto health = fetch_health();
    EXPECT_EQ(xgc::semantic::common::v1::PositioningHealth::POSITIONING_STATE_FROZEN,
              health.positioning().state());
    EXPECT_TRUE(has_fault(health, "mocap.position.frozen"));
    EXPECT_FALSE(has_fault(health, "mocap.vision.injection_held"));
    EXPECT_FALSE(has_fault(health, "mocap.attitude.divergence"));
    EXPECT_EQ("VRPN positioning data is frozen", health.summary());
  }

  // Regime 2: normal continuous motion (> 1.5 m cumulative without jumps).
  source.state();
  double cur_x = 0.0;
  for (int i = 1; i <= 10; ++i) {
    ros_t += 0.02;
    cur_x = 0.25 * static_cast<double>(i);
    source.localPose(cur_x, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
                     ros::Time(ros_t - 0.05));
    source.mocapPose(cur_x, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, ros::Time(ros_t));
  }
  expected_canonical += 10;
  expected_vision += 10;
  wait_ros(expected_canonical, expected_vision);
  EXPECT_FALSE(has_fault(fetch_health(), "mocap.vision.injection_held"));

  // Regime 3a: advancing-stamp pure static frames remain frozen without holding
  // vision output and keep canonical raw.
  source.state();
  for (int i = 0; i < 8; ++i) {
    ros_t += 0.02;
    source.mocapPose(cur_x, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, ros::Time(ros_t));
  }
  expected_canonical += 8;
  expected_vision += 8;
  wait_ros(expected_canonical, expected_vision);
  {
    const auto health = fetch_health();
    EXPECT_EQ(xgc::semantic::common::v1::PositioningHealth::POSITIONING_STATE_FROZEN,
              health.positioning().state());
    EXPECT_TRUE(has_fault(health, "mocap.position.frozen"));
    EXPECT_FALSE(has_fault(health, "mocap.vision.injection_held"));
    EXPECT_EQ("VRPN positioning data is frozen", health.summary());
  }

  // Regime 3b: duplicate timestamp, single out-of-order rejection, and clean
  // 5-frame clock rollback reset (no +0.01 m workaround).
  source.state();
  std::int64_t zenoh_t = 100000000000LL;
  source.publishZenohSample(cur_x, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, zenoh_t);
  expected_canonical += 1;
  expected_vision += 1;
  wait_ros(expected_canonical, expected_vision);

  for (int i = 0; i < 2; ++i) {
    source.publishZenohSample(cur_x, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, zenoh_t);
  }
  expected_canonical += 2;
  wait_ros(expected_canonical, expected_vision);
  EXPECT_TRUE(has_fault(fetch_health(), "mocap.vision.injection_held"));

  for (int i = 1; i <= 4; ++i) {
    zenoh_t += 20000000LL;
    source.publishZenohSample(cur_x, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, zenoh_t);
  }
  expected_canonical += 4;
  wait_ros(expected_canonical, expected_vision);
  EXPECT_TRUE(has_fault(fetch_health(), "mocap.vision.injection_held"));

  zenoh_t += 20000000LL;
  source.publishZenohSample(cur_x, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, zenoh_t);
  expected_canonical += 1;
  expected_vision += 1;
  wait_ros(expected_canonical, expected_vision);
  EXPECT_FALSE(has_fault(fetch_health(), "mocap.vision.injection_held"));

  // Single out-of-order Zenoh packet is rejected without lowering 100.10 s,
  // then 5 advancing frames at 100.12..100.20 s resume.
  source.publishZenohSample(cur_x, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
                            zenoh_t - 20000000LL);
  expected_canonical += 1;
  wait_ros(expected_canonical, expected_vision);
  EXPECT_TRUE(has_fault(fetch_health(), "mocap.vision.injection_held"));
  for (int i = 1; i <= 4; ++i) {
    zenoh_t += 20000000LL;
    source.publishZenohSample(cur_x, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, zenoh_t);
  }
  expected_canonical += 4;
  wait_ros(expected_canonical, expected_vision);
  zenoh_t += 20000000LL;
  source.publishZenohSample(cur_x, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, zenoh_t);
  expected_canonical += 1;
  expected_vision += 1;
  wait_ros(expected_canonical, expected_vision);
  EXPECT_FALSE(has_fault(fetch_health(), "mocap.vision.injection_held"));

  // Real same-domain clock rollback to 50.00 s resets on the 5th consistent frame.
  std::int64_t rb_t = 50000000000LL;
  for (int i = 0; i < 4; ++i) {
    source.publishZenohSample(cur_x, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, rb_t);
    rb_t += 20000000LL;
  }
  expected_canonical += 4;
  wait_ros(expected_canonical, expected_vision);
  EXPECT_TRUE(has_fault(fetch_health(), "mocap.vision.injection_held"));

  source.publishZenohSample(cur_x, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, rb_t);
  expected_canonical += 1;
  expected_vision += 1;
  wait_ros(expected_canonical, expected_vision);
  EXPECT_FALSE(has_fault(fetch_health(), "mocap.vision.injection_held"));

  rb_t += 20000000LL;
  source.publishZenohSample(cur_x, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, rb_t);
  expected_canonical += 1;
  expected_vision += 1;
  wait_ros(expected_canonical, expected_vision);

  // Regime 4: 115 ms VRPN timeout emits timeout + injection_held, then
  // immediately resumes publishing on the very first smooth stationary frame.
  ros_t += 0.02;
  source.mocapPose(cur_x, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, ros::Time(ros_t));
  expected_canonical += 1;
  expected_vision += 1;
  wait_ros(expected_canonical, expected_vision);

  std::this_thread::sleep_for(std::chrono::milliseconds(115));
  source.state();
  source.localPose(cur_x, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, ros::Time(ros_t));
  {
    const auto health = fetch_health();
    EXPECT_EQ(
        xgc::semantic::common::v1::PositioningHealth::POSITIONING_STATE_TIMED_OUT,
        health.positioning().state());
    EXPECT_TRUE(has_fault(health, "mocap.position.timeout"));
    EXPECT_TRUE(has_fault(health, "mocap.vision.injection_held"));
  }
  ros_t += 0.2;
  source.mocapPose(cur_x, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, ros::Time(ros_t));
  expected_canonical += 1;
  expected_vision += 1;
  wait_ros(expected_canonical, expected_vision);
  EXPECT_FALSE(has_fault(fetch_health(), "mocap.vision.injection_held"));

  // Regime 5: 115 ms timeout + 4.443 m jump + 8 pure static frames:
  // frames 0..4 hold (with FROZEN not masking injection_held in summary),
  // frames 5..8 publish cleanly while POSITIONING_STATE_FROZEN.
  std::this_thread::sleep_for(std::chrono::milliseconds(115));
  source.state();
  ros_t += 0.15;
  source.localPose(cur_x, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
                   ros::Time(ros_t - 0.05));
  cur_x += 4.443;
  source.mocapPose(cur_x, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, ros::Time(ros_t));
  expected_canonical += 1;
  wait_ros(expected_canonical, expected_vision);
  EXPECT_NEAR(4.443, fetch_latest_error(), 1e-6);
  EXPECT_TRUE(has_fault(fetch_health(), "mocap.vision.injection_held"));

  for (int i = 1; i <= 4; ++i) {
    ros_t += 0.02;
    source.mocapPose(cur_x, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, ros::Time(ros_t));
  }
  expected_canonical += 4;
  wait_ros(expected_canonical, expected_vision);
  EXPECT_TRUE(has_fault(fetch_health(), "mocap.vision.injection_held"));

  // Frames 5..8 at identical (cur_x, 0, 1) wash out the pre-jump window so
  // positioning becomes FROZEN while frame 5 releases and 6..8 also publish.
  for (int i = 5; i <= 8; ++i) {
    ros_t += 0.02;
    source.mocapPose(cur_x, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, ros::Time(ros_t));
  }
  expected_canonical += 4;
  expected_vision += 4;
  wait_ros(expected_canonical, expected_vision);
  {
    std::lock_guard<std::mutex> lock(ros_mutex);
    EXPECT_NEAR(cur_x, vision_poses.back().pose.position.x, 1e-9);
    EXPECT_NEAR(1.0, vision_poses.back().pose.position.z, 1e-9);
  }
  {
    const auto health = fetch_health();
    EXPECT_EQ(
        xgc::semantic::common::v1::PositioningHealth::POSITIONING_STATE_FROZEN,
        health.positioning().state());
    EXPECT_TRUE(has_fault(health, "mocap.position.frozen"));
    EXPECT_FALSE(has_fault(health, "mocap.vision.injection_held"));
    EXPECT_EQ("VRPN positioning data is frozen", health.summary());
  }

  // While FROZEN, triggering an active gate hold (repeated ROS stamp) must
  // surface "MAVROS vision_pose injection is held" instead of being masked by FROZEN.
  source.mocapPose(cur_x, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, ros::Time(ros_t));
  expected_canonical += 1;
  wait_ros(expected_canonical, expected_vision);
  {
    const auto health = fetch_health();
    EXPECT_EQ(
        xgc::semantic::common::v1::PositioningHealth::POSITIONING_STATE_FROZEN,
        health.positioning().state());
    EXPECT_TRUE(has_fault(health, "mocap.position.frozen"));
    EXPECT_TRUE(has_fault(health, "mocap.vision.injection_held"));
    EXPECT_EQ("MAVROS vision_pose injection is held", health.summary());
  }
  for (int i = 1; i <= 4; ++i) {
    ros_t += 0.02;
    source.mocapPose(cur_x, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, ros::Time(ros_t));
  }
  ros_t += 0.02;
  source.mocapPose(cur_x, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, ros::Time(ros_t));
  expected_canonical += 5;
  expected_vision += 1;
  wait_ros(expected_canonical, expected_vision);
  EXPECT_FALSE(has_fault(fetch_health(), "mocap.vision.injection_held"));

  // Regime 6: 0.0 m XYZ error + 179/172/178 deg flips while FC is online,
  // recovering cleanly on 5 stationary aligned frames.
  source.state();
  const double flip_angles[] = {179.0, 172.0, 178.0};
  for (const double angle_deg : flip_angles) {
    source.demand({});
    source.demand({"state.vision.pose"});
    ros_t += 0.02;
    const double half = (angle_deg * kPi / 180.0) * 0.5;
    source.localPose(cur_x, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
                     ros::Time(ros_t - 0.05));
    const std::size_t before_flip = source.messages().size();
    source.mocapPose(cur_x, 0.0, 1.0, 0.0, 0.0, std::sin(half), std::cos(half),
                     ros::Time(ros_t));
    expected_canonical += 1;
    wait_ros(expected_canonical, expected_vision);
    const auto frames = source.messages();
    for (std::size_t idx = before_flip; idx < frames.size(); ++idx) {
      EXPECT_NE("state.vision.pose", frames[idx].channel_id());
    }
    EXPECT_DOUBLE_EQ(0.0, fetch_latest_error());
    const auto health = fetch_health();
    EXPECT_TRUE(has_fault(health, "mocap.attitude.divergence"));
    EXPECT_TRUE(has_fault(health, "mocap.vision.injection_held"));
  }

  for (int i = 0; i < 5; ++i) {
    ros_t += 0.02;
    source.localPose(cur_x, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
                     ros::Time(ros_t - 0.05));
    source.mocapPose(cur_x, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, ros::Time(ros_t));
  }
  expected_canonical += 5;
  wait_ros(expected_canonical, expected_vision);
  {
    const auto health = fetch_health();
    EXPECT_TRUE(has_fault(health, "mocap.attitude.divergence"));
    EXPECT_TRUE(has_fault(health, "mocap.vision.injection_held"));
  }

  ros_t += 0.02;
  source.localPose(cur_x, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0,
                   ros::Time(ros_t - 0.05));
  source.mocapPose(cur_x, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, ros::Time(ros_t));
  expected_canonical += 1;
  expected_vision += 1;
  wait_ros(expected_canonical, expected_vision);
  {
    std::lock_guard<std::mutex> lock(ros_mutex);
    EXPECT_NEAR(0.0, vision_poses.back().pose.orientation.z, 1e-12);
    EXPECT_NEAR(1.0, vision_poses.back().pose.orientation.w, 1e-12);
    EXPECT_NEAR(cur_x, vision_poses.back().pose.position.x, 1e-9);
  }
  {
    const auto health = fetch_health();
    EXPECT_FALSE(has_fault(health, "mocap.attitude.divergence"));
    EXPECT_FALSE(has_fault(health, "mocap.vision.injection_held"));
  }

  // Regime 7: 20 repeated cycles of timeout resume + jump/static recovery
  // complete without deadlock accumulation.
  for (int round = 0; round < 20; ++round) {
    std::this_thread::sleep_for(std::chrono::milliseconds(115));
    source.state();
    ros_t += 0.15;
    source.mocapPose(cur_x, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, ros::Time(ros_t));
    expected_canonical += 1;
    expected_vision += 1;

    ros_t += 0.02;
    cur_x += 3.0;
    source.mocapPose(cur_x, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, ros::Time(ros_t));
    expected_canonical += 1;
    for (int f = 1; f <= 4; ++f) {
      ros_t += 0.02;
      source.mocapPose(cur_x, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, ros::Time(ros_t));
      expected_canonical += 1;
    }
    ros_t += 0.02;
    source.mocapPose(cur_x, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, ros::Time(ros_t));
    expected_canonical += 1;
    expected_vision += 1;
    wait_ros(expected_canonical, expected_vision);
  }
  EXPECT_FALSE(has_fault(fetch_health(), "mocap.vision.injection_held"));

  source.stop();
}

TEST(StatusDemand, DefaultOffProductionSafetyPublishesVisionWhileEmittingHonestDiagnostics) {
  constexpr double kPi = 3.14159265358979323846;
  auto build_vrpn = [](const std::string &profile_id) {
    xgc2_ros1_robot_adapter::RobotConfig cfg;
    cfg.robot_id = "prod_safe";
    cfg.profile_id = profile_id;
    cfg.parameters["namespace"] = "/prod_safe";
    cfg.parameters["mocap_rigid_body"] = "uav1";
    cfg.parameters["mocap_source_root"] = "/vrpn_client_node";
    cfg.parameters["localization_offset_x"] = "0.0";
    cfg.parameters["localization_offset_y"] = "0.0";
    cfg.parameters["localization_offset_z"] = "0.0";
    cfg.parameters["positioning_frame_number"] = "5";
    cfg.parameters["positioning_comparison_threshold_m"] = "0.05";
    NativeProfileConfig native;
    std::string error;
    EXPECT_TRUE(BuildNativeProfileConfig(cfg, &native, &error)) << error;
    return native;
  };
  const auto physical_native = build_vrpn("px4-multirotor.physical.vrpn");
  const auto gazebo_native = build_vrpn("px4-multirotor.gazebo.vrpn");
  ASSERT_FALSE(physical_native.vision_injection_gate.enabled);
  ASSERT_FALSE(gazebo_native.vision_injection_gate.enabled);

  StatusDemandHarness source(true, "/default_off/canonical",
                             "/default_off/vision", {}, {}, 0.05,
                             physical_native.vision_injection_gate,
                             "prod_safe", "/default_off");
  ros::NodeHandle nh;
  std::mutex ros_mutex;
  std::vector<geometry_msgs::PoseStamped> vision_poses;
  ros::Subscriber vision_sub = nh.subscribe<geometry_msgs::PoseStamped>(
      "/default_off/vision", 50,
      [&](const geometry_msgs::PoseStamped::ConstPtr &msg) {
        std::lock_guard<std::mutex> lock(ros_mutex);
        vision_poses.push_back(*msg);
      });
  const auto connect_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < connect_deadline) {
    ros::spinOnce();
    if (vision_sub.getNumPublishers() > 0)
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_GT(vision_sub.getNumPublishers(), 0u);

  auto wait_vision = [&](std::size_t expected) {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < deadline) {
      ros::spinOnce();
      {
        std::lock_guard<std::mutex> lock(ros_mutex);
        if (vision_poses.size() >= expected)
          break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    std::lock_guard<std::mutex> lock(ros_mutex);
    EXPECT_EQ(expected, vision_poses.size());
  };
  auto has_fault = [](const xgc::semantic::common::v1::VehicleHealth &health,
                      const std::string &code) {
    for (const auto &fault : health.faults()) {
      if (fault.code() == code)
        return true;
    }
    return false;
  };
  auto fetch_health = [&]() {
    source.demand({});
    const std::size_t before = source.messages().size();
    source.demand({"state.health"});
    const auto frames = source.messages();
    xgc::semantic::common::v1::VehicleHealth health;
    for (std::size_t i = frames.size(); i > before; --i) {
      if (frames[i - 1].channel_id() == "state.health" &&
          health.ParseFromString(frames[i - 1].message().payload().value())) {
        return health;
      }
    }
    ADD_FAILURE() << "missing state.health";
    return health;
  };
  auto fetch_error = [&]() {
    source.demand({});
    const std::size_t before = source.messages().size();
    source.demand({"state.localization.error"});
    const auto frames = source.messages();
    for (std::size_t i = frames.size(); i > before; --i) {
      if (frames[i - 1].channel_id() == "state.localization.error") {
        xgc::semantic::common::v1::DistanceEstimate dist;
        if (dist.ParseFromString(frames[i - 1].message().payload().value()))
          return dist.meters();
      }
    }
    ADD_FAILURE() << "missing state.localization.error";
    return -1.0;
  };

  source.state();
  double t = 20.0;
  for (int i = 0; i < 6; ++i) {
    t += 0.02;
    source.localPose(0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, ros::Time(t - 0.05));
    source.mocapPose(0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, ros::Time(t));
  }
  wait_vision(6);

  // Default-off gate NEVER holds 4.443 m jump, still reports truthful error.
  t += 0.02;
  source.localPose(0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, ros::Time(t - 0.05));
  source.mocapPose(4.443, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, ros::Time(t));
  wait_vision(7);
  EXPECT_NEAR(4.443, fetch_error(), 1e-6);
  EXPECT_FALSE(has_fault(fetch_health(), "mocap.vision.injection_held"));

  // Default-off gate NEVER holds 179 deg flip, still emits non-intrusive
  // mocap.attitude.divergence and keeps online == true.
  t += 0.02;
  const double half = (179.0 * kPi / 180.0) * 0.5;
  source.localPose(4.443, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, ros::Time(t - 0.05));
  source.mocapPose(4.443, 0.0, 1.0, 0.0, 0.0, std::sin(half), std::cos(half),
                   ros::Time(t));
  wait_vision(8);
  {
    const auto health = fetch_health();
    EXPECT_TRUE(health.online());
    EXPECT_TRUE(has_fault(health, "mocap.attitude.divergence"));
    EXPECT_FALSE(has_fault(health, "mocap.vision.injection_held"));
    EXPECT_EQ("Mocap attitude diverges from flight controller", health.summary());
  }
  source.stop();
}

TEST(StatusDemand, LiveRosMocapSubscriberQueueOneDropsIntermediateBursts) {
  VisionInjectionGateConfig gate_cfg;
  gate_cfg.enabled = false;
  StatusDemandHarness source(true, "/queue1/canonical", "/queue1/vision", {}, {},
                             0.05, gate_cfg, "queue1", "/queue1");

  ros::NodeHandle nh;
  ros::Publisher mocap_pub =
      nh.advertise<geometry_msgs::PoseStamped>("/queue1/mocap", 50);
  std::mutex ros_mutex;
  std::vector<geometry_msgs::PoseStamped> vision_poses;
  ros::Subscriber vision_sub = nh.subscribe<geometry_msgs::PoseStamped>(
      "/queue1/vision", 50,
      [&](const geometry_msgs::PoseStamped::ConstPtr &msg) {
        std::lock_guard<std::mutex> lock(ros_mutex);
        vision_poses.push_back(*msg);
      });

  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < deadline) {
    ros::spinOnce();
    if (mocap_pub.getNumSubscribers() > 0 && vision_sub.getNumPublishers() > 0)
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_GT(mocap_pub.getNumSubscribers(), 0u);
  ASSERT_GT(vision_sub.getNumPublishers(), 0u);

  // Publish 20 frames rapidly without spinning callbacks so roscpp's
  // queue_size=1 subscription drops stale backlog and delivers the newest pose.
  for (int i = 1; i <= 20; ++i) {
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = "world";
    msg.header.stamp = ros::Time(30.0 + 0.02 * i);
    msg.pose.position.x = static_cast<double>(i);
    msg.pose.position.z = 1.0;
    msg.pose.orientation.w = 1.0;
    mocap_pub.publish(msg);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(60));

  const auto pump_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < pump_deadline) {
    ros::spinOnce();
    {
      std::lock_guard<std::mutex> lock(ros_mutex);
      if (!vision_poses.empty() && vision_poses.back().pose.position.x >= 20.0)
        break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  ros::spinOnce();
  {
    std::lock_guard<std::mutex> lock(ros_mutex);
    ASSERT_FALSE(vision_poses.empty());
    EXPECT_LT(vision_poses.size(), 5u);
    EXPECT_NEAR(20.0, vision_poses.back().pose.position.x, 1e-9);
  }
  source.stop();
}

TEST(StatusDemand, MultiUavSingleProcessIsolatesVisionGatingAndHealth) {
  VisionInjectionGateConfig gate_cfg;
  gate_cfg.enabled = true;
  StatusDemandHarness uav1(true, "/uav1_src/canonical", "/uav1_src/vision", {},
                           {}, 0.05, gate_cfg, "uav1", "/uav1_src");
  StatusDemandHarness uav2(true, "/uav2_src/canonical", "/uav2_src/vision", {},
                           {}, 0.05, gate_cfg, "uav2", "/uav2_src");
  StatusDemandHarness uav3(true, "/uav3_src/canonical", "/uav3_src/vision", {},
                           {}, 0.05, gate_cfg, "uav3", "/uav3_src");

  ros::NodeHandle nh;
  std::mutex ros_mutex;
  std::vector<geometry_msgs::PoseStamped> v1, v2, v3;
  ros::Subscriber s1 = nh.subscribe<geometry_msgs::PoseStamped>(
      "/uav1_src/vision", 50,
      [&](const geometry_msgs::PoseStamped::ConstPtr &msg) {
        std::lock_guard<std::mutex> lock(ros_mutex);
        v1.push_back(*msg);
      });
  ros::Subscriber s2 = nh.subscribe<geometry_msgs::PoseStamped>(
      "/uav2_src/vision", 50,
      [&](const geometry_msgs::PoseStamped::ConstPtr &msg) {
        std::lock_guard<std::mutex> lock(ros_mutex);
        v2.push_back(*msg);
      });
  ros::Subscriber s3 = nh.subscribe<geometry_msgs::PoseStamped>(
      "/uav3_src/vision", 50,
      [&](const geometry_msgs::PoseStamped::ConstPtr &msg) {
        std::lock_guard<std::mutex> lock(ros_mutex);
        v3.push_back(*msg);
      });

  const auto connect_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < connect_deadline) {
    ros::spinOnce();
    if (s1.getNumPublishers() > 0 && s2.getNumPublishers() > 0 &&
        s3.getNumPublishers() > 0)
      break;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_GT(s1.getNumPublishers(), 0u);
  ASSERT_GT(s2.getNumPublishers(), 0u);
  ASSERT_GT(s3.getNumPublishers(), 0u);

  auto wait_counts = [&](std::size_t e1, std::size_t e2, std::size_t e3) {
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < deadline) {
      ros::spinOnce();
      {
        std::lock_guard<std::mutex> lock(ros_mutex);
        if (v1.size() >= e1 && v2.size() >= e2 && v3.size() >= e3)
          break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    ros::spinOnce();
    std::lock_guard<std::mutex> lock(ros_mutex);
    EXPECT_EQ(e1, v1.size());
    EXPECT_EQ(e2, v2.size());
    EXPECT_EQ(e3, v3.size());
  };
  auto fetch_health = [](StatusDemandHarness &h) {
    h.demand({});
    const std::size_t before = h.messages().size();
    h.demand({"state.health"});
    const auto frames = h.messages();
    xgc::semantic::common::v1::VehicleHealth out;
    for (std::size_t i = frames.size(); i > before; --i) {
      if (frames[i - 1].channel_id() == "state.health" &&
          out.ParseFromString(frames[i - 1].message().payload().value()))
        return out;
    }
    ADD_FAILURE() << "missing state.health";
    return out;
  };
  auto has_fault = [](const xgc::semantic::common::v1::VehicleHealth &health,
                      const std::string &code) {
    for (const auto &fault : health.faults()) {
      if (fault.code() == code)
        return true;
    }
    return false;
  };

  uav1.state();
  uav2.state();
  uav3.state();
  double t1 = 40.0, t2 = 40.0, t3 = 40.0;
  for (int i = 0; i < 6; ++i) {
    t1 += 0.02; t2 += 0.02; t3 += 0.02;
    uav1.mocapPose(1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, ros::Time(t1));
    uav2.mocapPose(2.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, ros::Time(t2));
    uav3.mocapPose(3.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, ros::Time(t3));
  }
  std::size_t e1 = 6, e2 = 6, e3 = 6;
  wait_counts(e1, e2, e3);

  // Silence uav1 for 120 ms while keeping uav2 and uav3 alive every 40 ms.
  for (int k = 0; k < 3; ++k) {
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    t2 += 0.04; t3 += 0.04;
    uav2.mocapPose(2.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, ros::Time(t2));
    uav3.mocapPose(3.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, ros::Time(t3));
    ++e2; ++e3;
  }
  wait_counts(e1, e2, e3);

  // Inject 4.443 m jump into uav2 while uav3 hovers cleanly and uav1 resumes.
  {
    const auto h1 = fetch_health(uav1);
    EXPECT_TRUE(h1.online());
    EXPECT_EQ(
        xgc::semantic::common::v1::PositioningHealth::POSITIONING_STATE_TIMED_OUT,
        h1.positioning().state());
    EXPECT_TRUE(has_fault(h1, "mocap.vision.injection_held"));
  }
  t1 += 0.15;
  uav1.mocapPose(1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, ros::Time(t1));
  ++e1; // Immediate reversible timeout resume on uav1!

  t2 += 0.02;
  uav2.mocapPose(2.0 + 4.443, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, ros::Time(t2));
  // uav2 holds while uav3 keeps publishing!
  t3 += 0.02;
  uav3.mocapPose(3.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, ros::Time(t3));
  ++e3;
  wait_counts(e1, e2, e3);

  const auto h1_resumed = fetch_health(uav1);
  const auto h2_held = fetch_health(uav2);
  const auto h3_ok = fetch_health(uav3);
  EXPECT_TRUE(h1_resumed.online());
  EXPECT_TRUE(h2_held.online());
  EXPECT_TRUE(h3_ok.online());
  EXPECT_FALSE(has_fault(h1_resumed, "mocap.vision.injection_held"));
  EXPECT_TRUE(has_fault(h2_held, "mocap.vision.injection_held"));
  EXPECT_FALSE(has_fault(h3_ok, "mocap.vision.injection_held"));

  for (int f = 1; f <= 4; ++f) {
    t2 += 0.02; t3 += 0.02;
    uav2.mocapPose(2.0 + 4.443, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, ros::Time(t2));
    uav3.mocapPose(3.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, ros::Time(t3));
    ++e3;
  }
  wait_counts(e1, e2, e3);

  t2 += 0.02; t3 += 0.02;
  uav2.mocapPose(2.0 + 4.443, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, ros::Time(t2));
  uav3.mocapPose(3.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, ros::Time(t3));
  ++e2; ++e3;
  wait_counts(e1, e2, e3);
  EXPECT_FALSE(has_fault(fetch_health(uav2), "mocap.vision.injection_held"));

  uav1.stop();
  uav2.stop();
  uav3.stop();
}

TEST(StatusDemand, StopJoinsAnOpeningSnapshotEmitter) {
  StatusDemandHarness source;
  source.pose(7);
  source.block();
  auto opening = std::async(std::launch::async, [&] { source.demand({"state.pose"}); });
  const bool entered = source.waitEntered();
  if (!entered) {
    source.release();
    opening.wait();
    FAIL() << "the opening snapshot did not reach its emitter";
  }
  auto stopping = std::async(std::launch::async, [&] { source.stop(); });
  EXPECT_EQ(std::future_status::timeout, stopping.wait_for(std::chrono::milliseconds(30)));
  source.release();
  opening.get();
  EXPECT_EQ(std::future_status::ready, stopping.wait_for(std::chrono::seconds(2)));
  stopping.get();
  const auto before = source.messages().size();
  source.pose(8);
  EXPECT_EQ(before, source.messages().size());
}
} // namespace xgc_px4_multirotor_ros1_adapter

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "px4_status_demand_test", ros::init_options::AnonymousName);
  const int result = RUN_ALL_TESTS();
  ros::shutdown();
  return result;
}
