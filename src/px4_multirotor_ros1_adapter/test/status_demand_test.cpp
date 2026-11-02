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
#include "xgc_px4_multirotor_ros1_adapter/generated_contract.hpp"
#include "xgc_px4_multirotor_ros1_adapter/robot_runtime.hpp"
#include "xgc/semantic/common/v1/telemetry.pb.h"
#include "xgc/semantic/aerial/v1/diagnostic.pb.h"

namespace xgc_px4_multirotor_ros1_adapter {
class StatusDemandHarness {
public:
  explicit StatusDemandHarness(bool live_sources = false) {
    ros::NodeHandle node;
    std::set<std::string> channels{"state.pose", "state.mocap.pose", "state.vision.pose", "state.velocity",
        "state.mocap.velocity", "state.mocap.acceleration", "state.mocap.speed", "state.localization.error",
        "state.imu", "state.power", "state.health", "state.flight", "state.controller", "setpoint.local",
        "setpoint.attitude", "diagnostic.fcu-link", "diagnostic.offboard-input", "diagnostic.stream-health"};
    NativeProfileConfig native;
    native.vision_publish_rate_hz = 30;
    if (live_sources) {
      const std::string prefix = "/demand_sources";
      native.pose_endpoint = prefix + "/pose";
      native.mocap_endpoint = prefix + "/mocap";
      native.mocap_velocity_endpoint = prefix + "/mocap_velocity";
      native.mocap_acceleration_endpoint = prefix + "/mocap_acceleration";
      native.canonical_pose_endpoint = prefix + "/canonical_pose";
      native.canonical_velocity_endpoint = prefix + "/canonical_velocity";
      native.canonical_acceleration_endpoint = prefix + "/canonical_acceleration";
      native.vision_pose_endpoint = prefix + "/vision";
      native.velocity_endpoint = prefix + "/velocity";
      native.imu_endpoint = prefix + "/imu";
      native.power_endpoint = prefix + "/power";
      native.controller_status_endpoint = prefix + "/controller";
      native.state_endpoint = prefix + "/state";
      native.extended_state_endpoint = prefix + "/extended";
      native.local_setpoint_endpoint = prefix + "/local_setpoint";
      native.attitude_setpoint_endpoint = prefix + "/attitude_setpoint";
      native.timesync_endpoint = prefix + "/timesync";
    }
    runtime_.reset(new RobotRuntime(std::move(node), "r1", InstalledContract().primaryProfile().profile_id,
        "/r1", channels, channels, std::move(native), {},
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
  void pose(double x) {
    geometry_msgs::PoseStamped::Ptr value(new geometry_msgs::PoseStamped);
    value->header.frame_id = "world";
    value->header.stamp = ros::Time(12, 3);
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
  void mocap(double x) {
    geometry_msgs::PoseStamped::Ptr value(new geometry_msgs::PoseStamped);
    value->header.frame_id = "world";
    value->pose.position.x = x;
    value->pose.orientation.w = 1;
    runtime_->mocapPoseCallback(value);
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
