#include <gtest/gtest.h>
#include <chrono>
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
#include "xgc_ugv_ros1_adapter/robot_runtime.hpp"
#include "xgc_ugv_ros1_adapter/variant.hpp"
#include "xgc/semantic/common/v1/telemetry.pb.h"
#include "xgc2/vrpn_zenoh/v1/tracker.pb.h"

namespace xgc_ugv_ros1_adapter {
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
  explicit StatusDemandHarness(bool live_sources = false, const UgvVariant &variant = scoutMini(),
                               const std::string &canonical_pose = "/demand_ugv/canonical_pose",
                               const std::string &mocap_body = "body",
                               const xgc2_ros1_robot_adapter::LocalizationProjectionConfig &localization = {},
                               const xgc2_ros1_robot_adapter::ZenohLocalizationConfig &zenoh = {}) {
    ros::NodeHandle node;
    std::set<std::string> channels{"vrpn.position", "command.velocity", "vrpn.velocity", "vrpn.acceleration",
        "vrpn.speed", "state.imu", "state.power", "state.health", "state.chassis", "state.controller",
        "state.reset", "diagnostic.stream-health"};
    const std::string prefix = live_sources ? "/demand_ugv" : "";
    runtime_.reset(new RobotRuntime(variant, std::move(node), "r1",
        variant.contract.primaryProfile().profile_id, "/r1", channels, channels,
        mocap_body, prefix + "/pose", prefix + "/velocity", prefix + "/accel",
        canonical_pose, prefix + "/canonical_velocity", prefix + "/canonical_accel", localization, zenoh,
        prefix + "/command", prefix + "/imu", prefix + "/battery", prefix + "/chassis",
        prefix + "/controller", prefix + "/session", prefix + "/result", {}, {},
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
      if (!runtime_->install(&error)) throw std::runtime_error(error);
    } else {
      std::lock_guard<std::mutex> lock(runtime_->mutex_);
      for (const auto &channel : channels)
        runtime_->ensureSourceLocked(channel, 5.0);
    }
  }

  ~StatusDemandHarness() {
    runtime_->Stop();
    runtime_.reset();
  }
  void pose(double x) {
    geometry_msgs::PoseStamped::Ptr value(new geometry_msgs::PoseStamped);
    value->header.frame_id = "world";
    value->header.stamp = ros::Time(12, 3);
    value->pose.position.x = x;
    value->pose.orientation.w = 1;
    runtime_->poseCallback(value);
  }
  void invalidPose() { pose(std::numeric_limits<double>::quiet_NaN()); }
  void imu() {
    sensor_msgs::Imu::Ptr value(new sensor_msgs::Imu);
    value->header.frame_id = "body";
    runtime_->imuCallback(value);
  }
  void chassis() {
    std_msgs::UInt32::Ptr value(new std_msgs::UInt32);
    value->data = packChassisState(1, 0, 0);
    runtime_->chassisStateCallback(value);
  }
  void seedAll() {
    pose(11);
    imu();
    chassis();
    geometry_msgs::Twist::Ptr command(new geometry_msgs::Twist);
    command->linear.x = 2;
    runtime_->commandVelocityCallback(command);
    geometry_msgs::TwistStamped::Ptr velocity(new geometry_msgs::TwistStamped);
    velocity->header.frame_id = "world";
    velocity->twist.linear.x = 3;
    runtime_->vrpnVelocityCallback(velocity);
    geometry_msgs::AccelStamped::Ptr acceleration(new geometry_msgs::AccelStamped);
    acceleration->header.frame_id = "world";
    acceleration->accel.linear.x = 4;
    runtime_->vrpnAccelerationCallback(acceleration);
    std_msgs::Float32::Ptr power(new std_msgs::Float32);
    power->data = 24;
    runtime_->voltageCallback(power);
    std_msgs::String::Ptr controller(new std_msgs::String);
    controller->data = "active";
    runtime_->controllerStatusCallback(controller);
    ugv_reset_msgs::ResetSession::Ptr session(new ugv_reset_msgs::ResetSession);
    session->session = 9;
    session->phase = ugv_reset_msgs::ResetSession::PHASE_MOVING;
    runtime_->resetSessionCallback(session);
    ugv_reset_msgs::ResetResult::Ptr result(new ugv_reset_msgs::ResetResult);
    result->session = 9;
    result->outcome = ugv_reset_msgs::ResetResult::OUTCOME_ARRIVED;
    result->detail = "arrived";
    runtime_->resetResultCallback(result);
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

TEST(StatusDemand, ZenohOpenReachesUgvCanonicalPose) {
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::NodeHandle node;
  const std::uint16_t port =
      static_cast<std::uint16_t>(20000u + (getpid() % 20000) + 3u);
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
  StatusDemandHarness source(true, scoutMini(), "/slot4/pose", "tracked_body",
                             localization, zenoh);
  std::atomic<unsigned> received{0};
  geometry_msgs::PoseStamped observed;
  auto canonical = node.subscribe<geometry_msgs::PoseStamped>("/slot4/pose", 10,
      [&](const geometry_msgs::PoseStamped::ConstPtr &message) {
        observed = *message; ++received;
      });
  ASSERT_TRUE(waitFor([&] {
    xgc2::vrpn_zenoh::v1::Pose sample;
    sample.mutable_report_time()->set_nanoseconds(123456789012345LL);
    sample.mutable_report_time()->set_clock_domain(xgc::v1::CLOCK_DOMAIN_SIMULATION);
    sample.mutable_position_m()->set_x(2.5);
    sample.mutable_orientation_xyzw()->set_w(1.0);
    publisher.publish(sample);
    return received.load() > 0;
  }));
  EXPECT_NEAR(3.75, observed.pose.position.x, 1e-12);
  EXPECT_EQ("world", observed.header.frame_id);
  source.stop();
  spinner.stop();
}

TEST(StatusDemand, ZenohZeroIntervalPreservesLinearSpeedWithoutFakeAngular) {
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::NodeHandle node;
  StatusDemandHarness source(true, scoutMini());
  source.pose(1.0);
  std::atomic<unsigned> canonical_velocity{0};
  auto twist = node.subscribe<geometry_msgs::TwistStamped>("/demand_ugv/canonical_velocity", 10,
      [&](const geometry_msgs::TwistStamped::ConstPtr &) { ++canonical_velocity; });
  ASSERT_TRUE(waitFor([&] { return twist.getNumPublishers() > 0; }));
  source.demand({"vrpn.speed", "vrpn.velocity"});
  source.zenohVelocity(3.0, 0.0, 0.0, 44);
  ASSERT_TRUE(waitFor([&] {
    for (const auto &item : source.messages())
      if (item.channel_id() == "vrpn.speed") return true;
    return false;
  }));
  bool speed_seen = false;
  for (const auto &item : source.messages()) {
    if (item.channel_id() == "vrpn.speed") {
      xgc::semantic::common::v1::SpeedEstimate speed;
      ASSERT_TRUE(speed.ParseFromString(item.message().payload().value()));
      EXPECT_NEAR(3.0, speed.meters_per_second(), 1e-12);
      speed_seen = true;
    }
    EXPECT_NE(item.channel_id(), "vrpn.velocity");
  }
  EXPECT_TRUE(speed_seen);
  EXPECT_EQ(0u, canonical_velocity.load());
  source.stop();
  spinner.stop();
}

TEST(StatusDemand, ZenohPoseUsesSlotOutputAndPreservesSourceTime) {
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::NodeHandle node;
  xgc2_ros1_robot_adapter::LocalizationProjectionConfig localization;
  localization.offset_x = 1.25;
  StatusDemandHarness source(true, scoutMini(), "/slot4/pose", "tracked_body", localization);
  std::atomic<unsigned> forwarded{0};
  geometry_msgs::PoseStamped observed;
  auto canonical = node.subscribe<geometry_msgs::PoseStamped>("/slot4/pose", 10,
      [&](const geometry_msgs::PoseStamped::ConstPtr &message) {
        observed = *message;
        ++forwarded;
      });
  ASSERT_TRUE(waitFor([&] { return canonical.getNumPublishers() > 0; }));
  source.zenohPose(2.5, 123456789012345LL);
  ASSERT_TRUE(waitFor([&] { return forwarded.load() > 0; }));
  EXPECT_NEAR(3.75, observed.pose.position.x, 1e-12);
  EXPECT_EQ("world", observed.header.frame_id);

  source.demand({"vrpn.position"});
  ASSERT_TRUE(waitFor([&] { return !source.messages().empty(); }));
  bool found = false;
  for (const auto &item : source.messages()) {
    if (item.channel_id() != "vrpn.position") continue;
    const auto &message = item.message();
    EXPECT_EQ(123456789012345LL, message.source_time().nanoseconds());
    EXPECT_EQ(xgc::v1::CLOCK_DOMAIN_SIMULATION,
              message.source_time().clock_domain());
    xgc::semantic::common::v1::PoseEstimate pose;
    ASSERT_TRUE(pose.ParseFromString(message.payload().value()));
    EXPECT_NEAR(3.75, pose.position().x(), 1e-12);
    found = true;
  }
  EXPECT_TRUE(found);
  source.stop();
  spinner.stop();
}

TEST(StatusDemand, ZenohVelocityAndAccelerationFeedCanonicalOutputs) {
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::NodeHandle node;
  StatusDemandHarness source(true, scoutMini());
  std::atomic<unsigned> twist_received{0}, accel_received{0};
  geometry_msgs::TwistStamped twist;
  geometry_msgs::AccelStamped accel;
  auto twist_sub = node.subscribe<geometry_msgs::TwistStamped>("/demand_ugv/canonical_velocity", 10,
      [&](const geometry_msgs::TwistStamped::ConstPtr &message) {
        twist = *message; ++twist_received;
      });
  auto accel_sub = node.subscribe<geometry_msgs::AccelStamped>("/demand_ugv/canonical_accel", 10,
      [&](const geometry_msgs::AccelStamped::ConstPtr &message) {
        accel = *message; ++accel_received;
      });
  ASSERT_TRUE(waitFor([&] { return twist_sub.getNumPublishers() > 0 && accel_sub.getNumPublishers() > 0; }));
  source.demand({"vrpn.velocity", "vrpn.acceleration"});
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

TEST(StatusDemand, ScoutRosSourcesFollowDemandWhileChassisAndPoseStayLive) {
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::NodeHandle node;
  StatusDemandHarness source(true, scoutMini());
  auto imu = node.advertise<sensor_msgs::Imu>("/demand_ugv/imu", 10, false);
  auto power = node.advertise<std_msgs::Float32>("/demand_ugv/battery", 10, false);
  auto command = node.advertise<geometry_msgs::Twist>("/demand_ugv/command", 10, false);
  auto chassis = node.advertise<std_msgs::UInt32>("/demand_ugv/chassis", 10, false);
  auto pose = node.advertise<geometry_msgs::PoseStamped>("/demand_ugv/pose", 10, false);
  auto reset = node.advertise<ugv_reset_msgs::ResetSession>("/demand_ugv/session", 10, false);
  std::atomic<unsigned> forwarded{0};
  auto canonical = node.subscribe<geometry_msgs::PoseStamped>("/demand_ugv/canonical_pose", 10,
      [&](const geometry_msgs::PoseStamped::ConstPtr &) { ++forwarded; });
  ASSERT_TRUE(waitFor([&] { return chassis.getNumSubscribers() && pose.getNumSubscribers() &&
                                  reset.getNumSubscribers() && canonical.getNumPublishers(); }));
  EXPECT_EQ(0u, imu.getNumSubscribers());
  EXPECT_EQ(0u, power.getNumSubscribers());
  EXPECT_EQ(0u, command.getNumSubscribers());
  std_msgs::UInt32 state;
  state.data = packChassisState(1, 0, 0);
  geometry_msgs::PoseStamped position;
  position.header.frame_id = "world";
  position.pose.orientation.w = 1;
  ASSERT_TRUE(waitFor([&] { chassis.publish(state); pose.publish(position);
                            return source.health().first && forwarded.load() > 0; }));
  EXPECT_TRUE(source.messages().empty());

  source.demand({"state.imu"});
  ASSERT_TRUE(waitFor([&] { return imu.getNumSubscribers() == 1; }));
  sensor_msgs::Imu sample;
  sample.header.frame_id = "old";
  ASSERT_TRUE(waitFor([&] { imu.publish(sample); return !source.messages().empty(); }));
  source.demand({"state.power"});
  ASSERT_TRUE(waitFor([&] { return !imu.getNumSubscribers() && power.getNumSubscribers() == 1; }));
  const auto before = source.messages().size();
  sample.header.frame_id = "unsubscribed";
  imu.publish(sample);
  source.demand({"state.imu"});
  ASSERT_TRUE(waitFor([&] { return imu.getNumSubscribers() == 1 && !power.getNumSubscribers(); }));
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(before, source.messages().size());
  sample.header.frame_id = "new";
  ASSERT_TRUE(waitFor([&] { imu.publish(sample); return source.messages().size() > before; }));
  xgc::semantic::common::v1::ImuEstimate observed;
  ASSERT_TRUE(observed.ParseFromString(source.messages().back().message().payload().value()));
  EXPECT_EQ("new", observed.frame_id());
  source.demand({"diagnostic.stream-health"});
  ASSERT_TRUE(waitFor([&] { return imu.getNumSubscribers() == 1 && power.getNumSubscribers() == 1 &&
                                  command.getNumSubscribers() == 1; }));
  source.demand({});
  ASSERT_TRUE(waitFor([&] { return !imu.getNumSubscribers() && !power.getNumSubscribers() &&
                                  !command.getNumSubscribers(); }));
  EXPECT_EQ(1u, chassis.getNumSubscribers());
  EXPECT_EQ(1u, pose.getNumSubscribers());
  EXPECT_EQ(1u, reset.getNumSubscribers());
  source.stop();
  spinner.stop();
}

TEST(StatusDemand, MecanumKeepsItsNativeImuHealthAtZeroDemand) {
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::NodeHandle node;
  StatusDemandHarness source(true, mecanumUgv());
  auto imu = node.advertise<sensor_msgs::Imu>("/demand_ugv/imu", 10, false);
  auto power = node.advertise<std_msgs::Float32>("/demand_ugv/battery", 10, false);
  ASSERT_TRUE(waitFor([&] { return imu.getNumSubscribers() == 1; }));
  EXPECT_EQ(0u, power.getNumSubscribers());
  sensor_msgs::Imu sample;
  sample.header.frame_id = "body";
  ASSERT_TRUE(waitFor([&] { imu.publish(sample); return source.health().first; }));
  EXPECT_TRUE(source.messages().empty());
  source.demand({"state.power"});
  ASSERT_TRUE(waitFor([&] { return power.getNumSubscribers() == 1; }));
  EXPECT_EQ(1u, imu.getNumSubscribers());
  source.demand({});
  ASSERT_TRUE(waitFor([&] { return !power.getNumSubscribers(); }));
  EXPECT_EQ(1u, imu.getNumSubscribers());
  source.stop();
  spinner.stop();
}

TEST(StatusDemand, ScoutRetirementAndStopJoinRealRosEmitter) {
  ros::AsyncSpinner spinner(2);
  spinner.start();
  ros::NodeHandle node;
  StatusDemandHarness source(true, scoutMini());
  auto imu = node.advertise<sensor_msgs::Imu>("/demand_ugv/imu", 10, false);
  source.demand({"state.imu"});
  ASSERT_TRUE(waitFor([&] { return imu.getNumSubscribers() == 1; }));
  source.block();
  sensor_msgs::Imu sample;
  sample.header.frame_id = "body";
  ASSERT_TRUE(waitFor([&] { imu.publish(sample); return source.waitEntered(); }));
  auto retirement = std::async(std::launch::async, [&] { source.demand({}); });
  EXPECT_EQ(std::future_status::timeout, retirement.wait_for(std::chrono::milliseconds(30)));
  auto stopping = std::async(std::launch::async, [&] { source.stop(); });
  EXPECT_EQ(std::future_status::timeout, stopping.wait_for(std::chrono::milliseconds(30)));
  source.release();
  ASSERT_EQ(std::future_status::ready, retirement.wait_for(std::chrono::seconds(3)));
  retirement.get();
  ASSERT_EQ(std::future_status::ready, stopping.wait_for(std::chrono::seconds(3)));
  stopping.get();
  EXPECT_TRUE(waitFor([&] { return !imu.getNumSubscribers(); }));
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
  source.chassis();
  source.periodic();
  ASSERT_TRUE(source.messages().empty());
  EXPECT_EQ(0u, source.encoded());
  const auto observed = source.rawObserved("vrpn.position");
  const auto health = source.health();
  EXPECT_TRUE(health.first);
  EXPECT_FALSE(health.second.empty());
  EXPECT_TRUE(source.messages().empty());
  source.demand({"vrpn.position"});
  auto frames = source.messages();
  ASSERT_EQ(1u, frames.size());
  xgc::semantic::common::v1::PoseEstimate pose;
  ASSERT_TRUE(pose.ParseFromString(frames[0].message().payload().value()));
  EXPECT_DOUBLE_EQ(2, pose.position().x());
  EXPECT_EQ(observed, static_cast<std::uint64_t>(frames[0].message().observed_unix_nanos()));
  source.imu();
  source.periodic();
  source.demand({"vrpn.position"});
  EXPECT_EQ(1u, source.messages().size());
  source.demand({});
  source.pose(3);
  source.imu();
  EXPECT_EQ(1u, source.messages().size());
  const auto latest_observed = source.rawObserved("vrpn.position");
  source.demand({"vrpn.position"});
  frames = source.messages();
  ASSERT_EQ(2u, frames.size());
  ASSERT_TRUE(pose.ParseFromString(frames[1].message().payload().value()));
  EXPECT_DOUBLE_EQ(3, pose.position().x());
  EXPECT_EQ(latest_observed, static_cast<std::uint64_t>(frames[1].message().observed_unix_nanos()));
  source.demand({});
}

TEST(StatusDemand, LastReleaseJoinsTheActualEmitter) {
  StatusDemandHarness source;
  source.demand({"vrpn.position"});
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

TEST(StatusDemand, EveryEnabledUgvChannelSnapshotsOnlyItsOwnRawSource) {
  StatusDemandHarness source;
  source.seedAll();
  ASSERT_TRUE(source.messages().empty());
  EXPECT_EQ(0u, source.encoded());
  for (const std::string channel : {"vrpn.position", "command.velocity", "vrpn.velocity", "vrpn.acceleration",
                                    "vrpn.speed", "state.imu", "state.power", "state.health", "state.chassis",
                                    "state.controller", "state.reset", "diagnostic.stream-health"}) {
    SCOPED_TRACE(channel);
    const auto before = source.messages().size();
    source.demand({channel});
    const auto frames = source.messages();
    ASSERT_EQ(before + 1, frames.size());
    EXPECT_EQ(channel, frames.back().channel_id());
    source.demand({});
  }
}

TEST(StatusDemand, InvalidLatestPoseDoesNotReplayAnOlderValidSample) {
  StatusDemandHarness source;
  source.pose(1);
  source.invalidPose();
  source.demand({"vrpn.position"});
  EXPECT_TRUE(source.messages().empty());
  source.demand({});
}

TEST(StatusDemand, InvalidLatestPoseDoesNotProduceSpeedFromAnOlderOrientation) {
  StatusDemandHarness source;
  source.seedAll();
  source.invalidPose();
  source.demand({"vrpn.speed"});
  EXPECT_TRUE(source.messages().empty());
  source.demand({});
  source.pose(5);
  source.demand({"vrpn.speed"});
  ASSERT_EQ(1u, source.messages().size());
  EXPECT_EQ("vrpn.speed", source.messages().back().channel_id());
  source.demand({});
}
TEST(StatusDemand, StopJoinsAnOpeningSnapshotEmitter) {
  StatusDemandHarness source;
  source.pose(7);
  source.block();
  auto opening = std::async(std::launch::async, [&] { source.demand({"vrpn.position"}); });
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
} // namespace xgc_ugv_ros1_adapter

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "ugv_status_demand_test", ros::init_options::AnonymousName);
  const int result = RUN_ALL_TESTS();
  ros::shutdown();
  return result;
}
