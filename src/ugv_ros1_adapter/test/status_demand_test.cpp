#include <gtest/gtest.h>
#include <chrono>
#include <condition_variable>
#include <future>
#include <limits>
#include <mutex>
#include <thread>
#include <vector>
#include <ros/ros.h>
#include "xgc_ugv_ros1_adapter/robot_runtime.hpp"
#include "xgc_ugv_ros1_adapter/variant.hpp"
#include "xgc/semantic/common/v1/telemetry.pb.h"

namespace xgc_ugv_ros1_adapter {
class StatusDemandHarness {
public:
  StatusDemandHarness() {
    ros::NodeHandle node;
    const auto &variant = scoutMini();
    std::set<std::string> channels{"vrpn.position", "command.velocity", "vrpn.velocity", "vrpn.acceleration",
        "vrpn.speed", "state.imu", "state.power", "state.health", "state.chassis", "state.controller",
        "state.reset", "diagnostic.stream-health"};
    runtime_.reset(new RobotRuntime(variant, std::move(node), "r1",
        variant.contract.primaryProfile().profile_id, "/r1", channels, channels,
        "body", "pose", "velocity", "accel", "pose", "velocity", "accel", {},
        "command", "imu", "battery", "chassis", "controller", "session", "result", {}, {},
        [this](xgc::robot::v1::RobotMessage message) {
          std::unique_lock<std::mutex> lock(mutex_);
          messages_.push_back(std::move(message));
          if (block_) {
            entered_ = true;
            changed_.notify_all();
            changed_.wait(lock, [this] { return released_; });
          }
        }));
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
