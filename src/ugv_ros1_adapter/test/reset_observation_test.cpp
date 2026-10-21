#include <gtest/gtest.h>

#include <cstdint>
#include <mutex>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <ros/ros.h>
#include <ugv_reset_msgs/ResetResult.h>
#include <ugv_reset_msgs/ResetSession.h>

#include "xgc/semantic/common/v1/telemetry.pb.h"
#include "xgc/semantic/ground/v1/reset.pb.h"
#include "xgc/v1/message.pb.h"
#include "xgc_ugv_ros1_adapter/robot_runtime.hpp"
#include "xgc_ugv_ros1_adapter/variant.hpp"

namespace xgc_ugv_ros1_adapter {

class ResetObservationHarness {
public:
  ResetObservationHarness() {
    ros::NodeHandle node;
    const UgvVariant &variant = scoutMini();
    const std::string profile = variant.contract.primaryProfile().profile_id;
    std::set<std::string> enabled{"state.reset", "diagnostic.stream-health"};
    std::set<std::string> required = enabled;
    contract::ChannelMetadata diagnostic{};
    if (!variant.contract.channelMetadata(profile, "diagnostic.stream-health",
                                          &diagnostic)) {
      throw std::logic_error("reset observation harness has no stream health");
    }
    for (std::size_t index = 0u; index < diagnostic.observes_count; ++index)
      required.insert(diagnostic.observes[index]);
    runtime_ = std::shared_ptr<RobotRuntime>(new RobotRuntime(
        variant, std::move(node), "ugv1", profile, "/ugv1", std::move(enabled),
        std::move(required), "body", "pose", "twist", "accel", "pose", "twist",
        "accel", {}, "command", "imu", "battery", "chassis", "status",
        "reset/session", "reset/result", {}, {},
        [this](xgc::robot::v1::RobotMessage message) {
          messages_.push_back(std::move(message));
        }));
    std::lock_guard<std::mutex> lock(runtime_->mutex_);
    for (std::size_t index = 0u; index < diagnostic.observes_count; ++index) {
      const std::string channel_id(diagnostic.observes[index]);
      contract::ChannelMetadata channel{};
      if (!variant.contract.channelMetadata(profile, channel_id, &channel) ||
          channel.stale_after_millis == 0u) {
        throw std::logic_error("generated channel stale policy is invalid");
      }
      runtime_->ensureSourceLocked(
          channel_id,
          static_cast<double>(channel.stale_after_millis) / 1000.0);
    }
  }

  void session(std::uint32_t id, std::uint8_t phase, const ros::Time &stamp,
               const ros::WallTime &now) {
    ugv_reset_msgs::ResetSession message;
    message.session = id;
    message.phase = phase;
    message.header.stamp = stamp;
    std::lock_guard<std::mutex> lock(runtime_->mutex_);
    runtime_->noteResetSessionLocked(message, now, &pending_);
    takePending();
  }

  void result(std::uint32_t id, std::uint8_t outcome, std::uint8_t reason,
              const ros::Time &stamp, const ros::WallTime &now) {
    ugv_reset_msgs::ResetResult message;
    message.session = id;
    message.outcome = outcome;
    message.reason = reason;
    message.final_planar_error = 0.061;
    message.final_yaw_error = 0.02;
    message.duration = 3.5;
    message.detail = "allowed to move but did not for 10 s";
    message.header.stamp = stamp;
    std::lock_guard<std::mutex> lock(runtime_->mutex_);
    runtime_->noteResetResultLocked(message, now, &pending_);
    takePending();
  }

  void diagnostic(const ros::WallTime &now) {
    std::vector<xgc::robot::v1::RobotMessage> messages;
    {
      std::lock_guard<std::mutex> lock(runtime_->mutex_);
      runtime_->emitStreamHealthLocked(now, &messages);
    }
    for (auto &message : messages)
      messages_.push_back(std::move(message));
  }

  std::uint64_t dropped() const {
    std::lock_guard<std::mutex> lock(runtime_->mutex_);
    const auto found = runtime_->sources_.find("state.reset");
    return found == runtime_->sources_.end() ? 0u : found->second.dropped_samples;
  }

  ros::WallTime lastSeen() const {
    std::lock_guard<std::mutex> lock(runtime_->mutex_);
    const auto found = runtime_->sources_.find("state.reset");
    return found == runtime_->sources_.end() ? ros::WallTime() : found->second.last_seen;
  }

  std::vector<xgc::semantic::ground::v1::ResetStatus> statuses() const {
    std::vector<xgc::semantic::ground::v1::ResetStatus> decoded;
    for (const auto &message : messages_) {
      if (message.channel_id() != "state.reset")
        continue;
      xgc::semantic::ground::v1::ResetStatus status;
      EXPECT_EQ("state.reset", message.channel_id());
      EXPECT_EQ(3105u, message.message().payload().schema().message_id());
      EXPECT_EQ("xgc.semantic.ground.v1.ResetStatus",
                message.message().payload().schema().type_name());
      EXPECT_TRUE(status.ParseFromString(message.message().payload().value()));
      decoded.push_back(status);
    }
    return decoded;
  }

  const xgc::robot::v1::RobotMessage &resetEnvelope(std::size_t index) const {
    std::size_t seen = 0u;
    for (const auto &message : messages_) {
      if (message.channel_id() != "state.reset")
        continue;
      if (seen == index)
        return message;
      ++seen;
    }
    throw std::out_of_range("reset envelope");
  }

  bool resetStale() const {
    for (auto it = messages_.rbegin(); it != messages_.rend(); ++it) {
      if (it->channel_id() != "diagnostic.stream-health")
        continue;
      xgc::semantic::common::v1::StreamHealthReport report;
      EXPECT_TRUE(report.ParseFromString(it->message().payload().value()));
      for (const auto &channel : report.channels()) {
        if (channel.channel_id() == "state.reset")
          return channel.stale();
      }
    }
    ADD_FAILURE() << "diagnostic report has no state.reset";
    return false;
  }

private:
  void takePending() {
    for (auto &message : pending_)
      messages_.push_back(std::move(message));
    pending_.clear();
  }

  std::shared_ptr<RobotRuntime> runtime_;
  std::vector<xgc::robot::v1::RobotMessage> messages_;
  std::vector<xgc::robot::v1::RobotMessage> pending_;
};

namespace {

TEST(ResetObservation, TimingContract) {
  ResetObservationHarness harness;
  const ros::WallTime t0(1000, 0);
  const ros::Time session_stamp(10, 0);
  const ros::Time result_stamp(20, 0);

  harness.result(1u, ugv_reset_msgs::ResetResult::OUTCOME_STOPPED, 0u,
                 result_stamp, t0);
  EXPECT_TRUE(harness.statuses().empty());
  EXPECT_TRUE(harness.lastSeen().isZero());

  harness.session(0u, ugv_reset_msgs::ResetSession::PHASE_IDLE, session_stamp, t0);
  harness.result(0u, 0u, 0u, result_stamp, t0);
  auto idle = harness.statuses();
  ASSERT_EQ(1u, idle.size());
  EXPECT_EQ(0u, idle[0].generation());
  EXPECT_EQ(xgc::semantic::ground::v1::RESET_PHASE_IDLE, idle[0].phase());
  EXPECT_FALSE(idle[0].has_result());
  EXPECT_EQ(session_stamp.toNSec(),
            harness.resetEnvelope(0).message().source_time().nanoseconds());
  EXPECT_EQ(xgc::v1::CLOCK_DOMAIN_NATIVE,
            harness.resetEnvelope(0).message().source_time().clock_domain());
  EXPECT_NE(session_stamp.toNSec(),
            harness.resetEnvelope(0).message().observed_unix_nanos());

  harness.result(1u, ugv_reset_msgs::ResetResult::OUTCOME_STOPPED, 0u,
                 result_stamp, ros::WallTime(1000, 1000000));
  EXPECT_EQ(1u, harness.statuses().size());
  const auto throttled = harness.dropped();
  EXPECT_EQ(1u, throttled);

  harness.session(1u, ugv_reset_msgs::ResetSession::PHASE_IDLE, session_stamp,
                  ros::WallTime(1000, 2000000));
  auto attached = harness.statuses();
  ASSERT_EQ(2u, attached.size());
  EXPECT_EQ(1u, attached[1].generation());
  EXPECT_EQ(xgc::semantic::ground::v1::RESET_PHASE_IDLE, attached[1].phase());
  ASSERT_TRUE(attached[1].has_result());
  EXPECT_EQ(xgc::semantic::ground::v1::RESET_OUTCOME_STOPPED,
            attached[1].result().outcome());
  EXPECT_EQ(result_stamp.toNSec(), attached[1].result().source_stamp_ns());
  EXPECT_EQ(session_stamp.toNSec(),
            harness.resetEnvelope(1).message().source_time().nanoseconds());
  EXPECT_NE(attached[1].result().source_stamp_ns(),
            harness.resetEnvelope(1).message().observed_unix_nanos());
  EXPECT_EQ(throttled, harness.dropped());

  harness.session(2u, ugv_reset_msgs::ResetSession::PHASE_REQUESTING,
                  ros::Time(11, 0), ros::WallTime(1000, 3000000));
  auto cleared = harness.statuses();
  ASSERT_EQ(attached.size() + 1u, cleared.size());
  EXPECT_EQ(2u, cleared.back().generation());
  EXPECT_EQ(xgc::semantic::ground::v1::RESET_PHASE_REQUESTING, cleared.back().phase());
  EXPECT_FALSE(cleared.back().has_result());
  EXPECT_EQ(throttled, harness.dropped());

  harness.result(2u, ugv_reset_msgs::ResetResult::OUTCOME_REJECTED_NO_COORDINATOR,
                 0u, result_stamp, ros::WallTime(1000, 8000000));
  auto ordered = harness.statuses();
  ASSERT_EQ(cleared.size() + 1u, ordered.size());
  ASSERT_TRUE(ordered.back().has_result());
  EXPECT_EQ(xgc::semantic::ground::v1::RESET_OUTCOME_REJECTED_NO_COORDINATOR,
            ordered.back().result().outcome());
  EXPECT_EQ(2u, ordered.back().generation());
  EXPECT_EQ(throttled, harness.dropped());

  const auto before_heartbeat = ordered.size();
  harness.session(2u, ugv_reset_msgs::ResetSession::PHASE_REQUESTING,
                  ros::Time(11, 0), ros::WallTime(1000, 10000000));
  EXPECT_EQ(before_heartbeat, harness.statuses().size());
  EXPECT_EQ(throttled + 1u, harness.dropped());
  harness.session(2u, ugv_reset_msgs::ResetSession::PHASE_REQUESTING,
                  ros::Time(11, 0), ros::WallTime(1000, 220000000));
  auto heartbeat = harness.statuses();
  EXPECT_EQ(before_heartbeat + 1u, heartbeat.size());
  EXPECT_EQ(2u, heartbeat.back().generation());
  EXPECT_TRUE(heartbeat.back().has_result());

  const ros::WallTime fresh(2000, 0);
  harness.session(2u, ugv_reset_msgs::ResetSession::PHASE_REQUESTING,
                  ros::Time(11, 0), fresh);
  harness.diagnostic(fresh);
  EXPECT_FALSE(harness.resetStale());
  const ros::WallTime stale(2002, 1000000);
  harness.diagnostic(stale);
  EXPECT_TRUE(harness.resetStale());
  const auto before_late_result = harness.statuses().size();
  const auto seen = harness.lastSeen();
  harness.result(2u, ugv_reset_msgs::ResetResult::OUTCOME_ARRIVED, 0u,
                 result_stamp, ros::WallTime(2003, 0));
  EXPECT_EQ(before_late_result, harness.statuses().size());
  EXPECT_EQ(seen, harness.lastSeen());
  harness.diagnostic(ros::WallTime(2003, 0));
  EXPECT_TRUE(harness.resetStale());
}

TEST(ResetObservation, SmallerNewSessionMatchesDespiteOlderLargerCounter) {
  ResetObservationHarness harness;
  const ros::Time stamp(10, 0);
  harness.session(1000u, ugv_reset_msgs::ResetSession::PHASE_IDLE, stamp,
                  ros::WallTime(5000, 0));
  harness.result(1000u, ugv_reset_msgs::ResetResult::OUTCOME_STOPPED, 0u, stamp,
                 ros::WallTime(5000, 1000000));
  harness.session(0u, ugv_reset_msgs::ResetSession::PHASE_IDLE, stamp,
                  ros::WallTime(5000, 2000000));
  harness.session(10u, ugv_reset_msgs::ResetSession::PHASE_IDLE, stamp,
                  ros::WallTime(5000, 3000000));
  harness.result(10u, ugv_reset_msgs::ResetResult::OUTCOME_ARRIVED, 0u, stamp,
                 ros::WallTime(5000, 4000000));
  const auto statuses = harness.statuses();
  ASSERT_FALSE(statuses.empty());
  EXPECT_EQ(10u, statuses.back().generation());
  ASSERT_TRUE(statuses.back().has_result());
  EXPECT_EQ(xgc::semantic::ground::v1::RESET_OUTCOME_ARRIVED,
            statuses.back().result().outcome());
}

TEST(ResetObservation, ResultArrivingBeforeItsSessionAttachesOnEquality) {
  ResetObservationHarness harness;
  const ros::Time stamp(10, 0);
  harness.session(1000u, ugv_reset_msgs::ResetSession::PHASE_IDLE, stamp,
                  ros::WallTime(5100, 0));
  harness.result(1000u, ugv_reset_msgs::ResetResult::OUTCOME_STOPPED, 0u, stamp,
                 ros::WallTime(5100, 1000000));
  harness.result(10u, ugv_reset_msgs::ResetResult::OUTCOME_ARRIVED, 0u, stamp,
                 ros::WallTime(5100, 2000000));
  auto before = harness.statuses();
  ASSERT_FALSE(before.empty());
  EXPECT_EQ(1000u, before.back().generation());
  ASSERT_TRUE(before.back().has_result());
  EXPECT_EQ(xgc::semantic::ground::v1::RESET_OUTCOME_STOPPED,
            before.back().result().outcome());
  harness.session(10u, ugv_reset_msgs::ResetSession::PHASE_IDLE, stamp,
                  ros::WallTime(5100, 3000000));
  auto attached = harness.statuses();
  EXPECT_EQ(10u, attached.back().generation());
  ASSERT_TRUE(attached.back().has_result());
  EXPECT_EQ(xgc::semantic::ground::v1::RESET_OUTCOME_ARRIVED,
            attached.back().result().outcome());
}

TEST(ResetObservation, UnrelatedResultsDoNotClearTheCurrentMatch) {
  ResetObservationHarness harness;
  const ros::Time stamp(10, 0);
  harness.session(10u, ugv_reset_msgs::ResetSession::PHASE_IDLE, stamp,
                  ros::WallTime(5200, 0));
  harness.result(10u, ugv_reset_msgs::ResetResult::OUTCOME_ARRIVED, 0u, stamp,
                 ros::WallTime(5200, 1000000));
  const auto matched = harness.statuses().size();
  harness.result(1000u, ugv_reset_msgs::ResetResult::OUTCOME_STOPPED, 0u, stamp,
                 ros::WallTime(5200, 2000000));
  harness.result(999u, ugv_reset_msgs::ResetResult::OUTCOME_STOPPED, 0u, stamp,
                 ros::WallTime(5200, 3000000));
  harness.session(10u, ugv_reset_msgs::ResetSession::PHASE_IDLE, stamp,
                  ros::WallTime(5200, 4000000));
  const auto statuses = harness.statuses();
  EXPECT_EQ(matched, statuses.size());
  EXPECT_EQ(10u, statuses.back().generation());
  ASSERT_TRUE(statuses.back().has_result());
  EXPECT_EQ(xgc::semantic::ground::v1::RESET_OUTCOME_ARRIVED,
            statuses.back().result().outcome());
}

TEST(ResetObservation, ResultBeforeSessionDoesNotRefreshFreshness) {
  ResetObservationHarness harness;
  const ros::Time stamp(10, 0);
  harness.result(10u, ugv_reset_msgs::ResetResult::OUTCOME_ARRIVED, 0u, stamp,
                 ros::WallTime(5300, 0));
  EXPECT_TRUE(harness.statuses().empty());
  EXPECT_TRUE(harness.lastSeen().isZero());
  harness.session(10u, ugv_reset_msgs::ResetSession::PHASE_IDLE, stamp,
                  ros::WallTime(5300, 1000000));
  const auto statuses = harness.statuses();
  ASSERT_FALSE(statuses.empty());
  EXPECT_EQ(10u, statuses.back().generation());
  ASSERT_TRUE(statuses.back().has_result());
  EXPECT_EQ(ros::WallTime(5300, 1000000), harness.lastSeen());
}

TEST(ResetObservation, OneCandidateRemainsUntilItsSessionArrives) {
  ResetObservationHarness harness;
  const ros::Time stamp(10, 0);
  harness.session(10u, ugv_reset_msgs::ResetSession::PHASE_IDLE, stamp,
                  ros::WallTime(5400, 0));
  harness.result(10u, ugv_reset_msgs::ResetResult::OUTCOME_ARRIVED, 0u, stamp,
                 ros::WallTime(5400, 1000000));
  harness.result(11u, ugv_reset_msgs::ResetResult::OUTCOME_STOPPED, 0u, stamp,
                 ros::WallTime(5400, 2000000));
  harness.session(10u, ugv_reset_msgs::ResetSession::PHASE_IDLE, stamp,
                  ros::WallTime(5400, 3000000));
  auto during = harness.statuses();
  EXPECT_EQ(10u, during.back().generation());
  ASSERT_TRUE(during.back().has_result());
  EXPECT_EQ(xgc::semantic::ground::v1::RESET_OUTCOME_ARRIVED,
            during.back().result().outcome());
  harness.session(11u, ugv_reset_msgs::ResetSession::PHASE_IDLE, stamp,
                  ros::WallTime(5400, 4000000));
  auto next = harness.statuses();
  EXPECT_EQ(11u, next.back().generation());
  ASSERT_TRUE(next.back().has_result());
  EXPECT_EQ(xgc::semantic::ground::v1::RESET_OUTCOME_STOPPED,
            next.back().result().outcome());
}

TEST(ResetObservation, ASecondUnmatchedCandidateReplacesTheEarlierOne) {
  ResetObservationHarness harness;
  const ros::Time stamp(10, 0);
  harness.session(10u, ugv_reset_msgs::ResetSession::PHASE_IDLE, stamp,
                  ros::WallTime(5500, 0));
  harness.result(10u, ugv_reset_msgs::ResetResult::OUTCOME_ARRIVED, 0u, stamp,
                 ros::WallTime(5500, 1000000));
  harness.result(11u, ugv_reset_msgs::ResetResult::OUTCOME_STOPPED, 0u, stamp,
                 ros::WallTime(5500, 2000000));
  harness.result(12u, ugv_reset_msgs::ResetResult::OUTCOME_STOPPED, 0u, stamp,
                 ros::WallTime(5500, 3000000));
  harness.session(11u, ugv_reset_msgs::ResetSession::PHASE_IDLE, stamp,
                  ros::WallTime(5500, 4000000));
  const auto statuses = harness.statuses();
  EXPECT_EQ(11u, statuses.back().generation());
  EXPECT_FALSE(statuses.back().has_result());
}

TEST(ResetObservation, LaterSessionProjectsTheLatestArrival) {
  ResetObservationHarness harness;
  const ros::Time stamp(10, 0);
  harness.session(1000u, ugv_reset_msgs::ResetSession::PHASE_IDLE, stamp,
                  ros::WallTime(5600, 0));
  harness.result(1000u, ugv_reset_msgs::ResetResult::OUTCOME_STOPPED, 0u, stamp,
                 ros::WallTime(5600, 1000000));
  harness.session(10u, ugv_reset_msgs::ResetSession::PHASE_IDLE, stamp,
                  ros::WallTime(5600, 2000000));
  harness.result(10u, ugv_reset_msgs::ResetResult::OUTCOME_ARRIVED, 0u, stamp,
                 ros::WallTime(5600, 3000000));
  harness.session(1000u, ugv_reset_msgs::ResetSession::PHASE_IDLE, stamp,
                  ros::WallTime(5600, 4000000));
  const auto statuses = harness.statuses();
  EXPECT_EQ(1000u, statuses.back().generation());
  ASSERT_TRUE(statuses.back().has_result());
  EXPECT_EQ(xgc::semantic::ground::v1::RESET_OUTCOME_STOPPED,
            statuses.back().result().outcome());
}

TEST(ResetObservation, SameIdCollisionStaysUnresolved) {
  ResetObservationHarness harness;
  const ros::Time stamp(10, 0);
  harness.session(7u, ugv_reset_msgs::ResetSession::PHASE_IDLE, stamp,
                  ros::WallTime(5700, 0));
  harness.result(7u, ugv_reset_msgs::ResetResult::OUTCOME_ARRIVED, 0u, stamp,
                 ros::WallTime(5700, 1000000));
  harness.session(0u, ugv_reset_msgs::ResetSession::PHASE_IDLE, stamp,
                  ros::WallTime(5700, 2000000));
  harness.session(7u, ugv_reset_msgs::ResetSession::PHASE_REQUESTING, stamp,
                  ros::WallTime(5700, 3000000));
  // Equality cannot tell a reordered packet of session 7 from a new
  // controller that reused 7. An attached result is that collision.
  // It is not acceptance that a new session arrived.
  const auto statuses = harness.statuses();
  ASSERT_FALSE(statuses.empty());
  EXPECT_EQ(7u, statuses.back().generation());
  EXPECT_EQ(xgc::semantic::ground::v1::RESET_PHASE_REQUESTING,
            statuses.back().phase());
  EXPECT_TRUE(statuses.back().has_result());
}

TEST(ResetObservation, InvalidResultsDoNotOccupyOrClear) {
  ResetObservationHarness harness;
  const ros::Time stamp(10, 0);
  const ros::WallTime seen(5800, 0);
  harness.session(10u, ugv_reset_msgs::ResetSession::PHASE_IDLE, stamp, seen);
  harness.result(10u, ugv_reset_msgs::ResetResult::OUTCOME_ARRIVED, 0u, stamp,
                 ros::WallTime(5800, 1000000));
  const auto before = harness.statuses().size();
  harness.result(0u, ugv_reset_msgs::ResetResult::OUTCOME_STOPPED, 0u, stamp,
                 ros::WallTime(5800, 2000000));
  harness.result(11u, 0u, 0u, stamp, ros::WallTime(5800, 3000000));
  const auto statuses = harness.statuses();
  EXPECT_EQ(before, statuses.size());
  EXPECT_EQ(seen, harness.lastSeen());
  EXPECT_EQ(10u, statuses.back().generation());
  ASSERT_TRUE(statuses.back().has_result());
  EXPECT_EQ(xgc::semantic::ground::v1::RESET_OUTCOME_ARRIVED,
            statuses.back().result().outcome());
}

TEST(ResetObservation, StaleResultDoesNotRefreshAndAFreshSessionRematches) {
  ResetObservationHarness harness;
  const ros::Time stamp(10, 0);
  const ros::WallTime seen(5900, 0);
  harness.session(10u, ugv_reset_msgs::ResetSession::PHASE_IDLE, stamp, seen);
  harness.result(10u, ugv_reset_msgs::ResetResult::OUTCOME_ARRIVED, 0u, stamp,
                 ros::WallTime(5900, 1000000));
  const auto before = harness.statuses().size();
  harness.diagnostic(ros::WallTime(5902, 1000000));
  EXPECT_TRUE(harness.resetStale());
  harness.result(10u, ugv_reset_msgs::ResetResult::OUTCOME_ARRIVED, 0u, stamp,
                 ros::WallTime(5902, 2000000));
  EXPECT_EQ(before, harness.statuses().size());
  EXPECT_EQ(seen, harness.lastSeen());
  harness.session(10u, ugv_reset_msgs::ResetSession::PHASE_IDLE, stamp,
                  ros::WallTime(5903, 0));
  const auto statuses = harness.statuses();
  EXPECT_EQ(10u, statuses.back().generation());
  ASSERT_TRUE(statuses.back().has_result());
  EXPECT_EQ(xgc::semantic::ground::v1::RESET_OUTCOME_ARRIVED,
            statuses.back().result().outcome());
  EXPECT_EQ(ros::WallTime(5903, 0), harness.lastSeen());
}

} // namespace
} // namespace xgc_ugv_ros1_adapter

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "reset_observation_test");
  const int result = RUN_ALL_TESTS();
  ros::shutdown();
  return result;
}
