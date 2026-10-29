#include <gtest/gtest.h>
#include <chrono>
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
  StatusDemandHarness() {
    ros::NodeHandle node;
    std::set<std::string> channels{"state.pose", "state.mocap.pose", "state.vision.pose", "state.velocity",
        "state.mocap.velocity", "state.mocap.acceleration", "state.mocap.speed", "state.localization.error",
        "state.imu", "state.power", "state.health", "state.flight", "state.controller", "setpoint.local",
        "setpoint.attitude", "diagnostic.fcu-link", "diagnostic.offboard-input", "diagnostic.stream-health"};
    NativeProfileConfig native;
    native.vision_publish_rate_hz = 30;
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
    runtime_->vision_pose_publisher_ = runtime_->node_handle_.advertise<geometry_msgs::PoseStamped>("status_test_vision", 1);
    std::lock_guard<std::mutex> lock(runtime_->mutex_);
    for (const auto &channel : channels)
      runtime_->ensureSourceLocked(channel, 5.0);
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
  void periodic() { runtime_->emitPeriodic(ros::WallTime::now()); }
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
