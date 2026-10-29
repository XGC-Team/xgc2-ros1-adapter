#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <stdexcept>
#include <vector>

#include "xgc_px4_multirotor_ros1_adapter/remote_control.hpp"

namespace xgc_px4_multirotor_ros1_adapter {
namespace {

TEST(RemoteControlPublisher, ZeroIntentHoldsAltitudeForOffboard) {
  ros::Time::init();
  std::vector<mavros_msgs::PositionTarget> published;
  RemoteControlPublisher publisher(
      [&published](const mavros_msgs::PositionTarget &target) {
        published.push_back(target);
      },
      1.0, 1.5, 0.9);
  std::string error;

  ASSERT_TRUE(publisher.SetIntent(1, 0, 0, 0, &error)) << error;
  ASSERT_EQ(1u, published.size());
  EXPECT_DOUBLE_EQ(1.0, published.back().position.z);
  EXPECT_DOUBLE_EQ(0.0, published.back().velocity.x);
  EXPECT_DOUBLE_EQ(0.0, published.back().velocity.y);
  EXPECT_DOUBLE_EQ(0.0, published.back().yaw_rate);
  EXPECT_EQ(mavros_msgs::PositionTarget::FRAME_LOCAL_NED,
            published.back().coordinate_frame);
  EXPECT_EQ(mavros_msgs::PositionTarget::IGNORE_PX |
                mavros_msgs::PositionTarget::IGNORE_PY |
                mavros_msgs::PositionTarget::IGNORE_VZ |
                mavros_msgs::PositionTarget::IGNORE_AFX |
                mavros_msgs::PositionTarget::IGNORE_AFY |
                mavros_msgs::PositionTarget::IGNORE_AFZ |
                mavros_msgs::PositionTarget::IGNORE_YAW,
            published.back().type_mask);
  publisher.PublishPeriodic();
  ASSERT_EQ(2u, published.size());
  EXPECT_DOUBLE_EQ(1.0, published.back().position.z);
  EXPECT_DOUBLE_EQ(0.0, published.back().velocity.x);

  ASSERT_TRUE(publisher.Release(&error)) << error;
  ASSERT_EQ(3u, published.size());
  EXPECT_DOUBLE_EQ(0.0, published.back().velocity.x);
  publisher.PublishPeriodic();
  EXPECT_EQ(3u, published.size());
}

TEST(RemoteControlPublisher, ReleaseSubmitsZeroAndReportsPublicationFailure) {
  ros::Time::init();
  bool fail_publish = false;
  std::vector<mavros_msgs::PositionTarget> published;
  RemoteControlPublisher publisher(
      [&](const mavros_msgs::PositionTarget &target) {
        if (fail_publish) throw std::runtime_error("transport down");
        published.push_back(target);
      },
      1.0, 1.5, 0.9);
  std::string error;
  ASSERT_TRUE(publisher.SetIntent(3, 1, -1, 1, &error)) << error;
  ASSERT_EQ(1u, published.size());
  EXPECT_DOUBLE_EQ(1.5, published.back().velocity.x);

  fail_publish = true;
  EXPECT_FALSE(publisher.Release(&error));
  EXPECT_FALSE(error.empty());
  EXPECT_EQ(1u, published.size());

  fail_publish = false;
  ASSERT_TRUE(publisher.Release(&error)) << error;
  ASSERT_EQ(2u, published.size());
  EXPECT_DOUBLE_EQ(0.0, published.back().velocity.x);
  EXPECT_DOUBLE_EQ(0.0, published.back().velocity.y);
  EXPECT_DOUBLE_EQ(0.0, published.back().yaw_rate);
  EXPECT_DOUBLE_EQ(1.0, published.back().position.z);
  publisher.PublishPeriodic();
  EXPECT_EQ(2u, published.size());
}

TEST(RemoteControlPublisher, MapsDiscreteIntentAndStopsWithOneMetreHold) {
  ros::Time::init();
  std::vector<mavros_msgs::PositionTarget> published;
  RemoteControlPublisher publisher(
      [&published](const mavros_msgs::PositionTarget &target) {
        published.push_back(target);
      },
      1.0, 1.5, 0.9);
  std::string error;

  ASSERT_TRUE(publisher.SetIntent(2, 1, -1, 1, &error)) << error;
  ASSERT_EQ(1u, published.size());
  EXPECT_DOUBLE_EQ(1.0, published.back().position.z);
  EXPECT_DOUBLE_EQ(1.0, published.back().velocity.x);
  EXPECT_DOUBLE_EQ(-1.0, published.back().velocity.y);
  EXPECT_NEAR(0.6, published.back().yaw_rate, 1e-6);
  EXPECT_EQ(mavros_msgs::PositionTarget::FRAME_LOCAL_NED,
            published.back().coordinate_frame);

  publisher.PublishPeriodic();
  ASSERT_EQ(2u, published.size());
  publisher.Stop();
  ASSERT_EQ(3u, published.size());
  EXPECT_DOUBLE_EQ(0.0, published.back().velocity.x);
  EXPECT_DOUBLE_EQ(0.0, published.back().velocity.y);
  EXPECT_DOUBLE_EQ(0.0, published.back().yaw_rate);
  EXPECT_DOUBLE_EQ(1.0, published.back().position.z);
  publisher.PublishPeriodic();
  EXPECT_EQ(3u, published.size());
}

using std::chrono::milliseconds;

// Dead-man: Core has been silent for the limit; the stream ends with one zero
// setpoint and does not restart by itself.
TEST(RemoteControlPublisher, CoreSilenceReleasesAnActiveIntentAfterTheLimit) {
  ros::Time::init();
  std::vector<mavros_msgs::PositionTarget> published;
  RemoteControlPublisher publisher(
      [&published](const mavros_msgs::PositionTarget &target) {
        published.push_back(target);
      },
      1.0, 1.5, 0.9, milliseconds(3000));
  std::string error;
  ASSERT_TRUE(publisher.SetIntent(3, 1, 0, 0, &error)) << error;
  EXPECT_DOUBLE_EQ(1.5, published.back().velocity.x);

  // Below the limit the intent keeps being republished.
  EXPECT_FALSE(publisher.CoreSilent(milliseconds(2999)));
  publisher.PublishPeriodic();
  ASSERT_EQ(2u, published.size());
  EXPECT_DOUBLE_EQ(1.5, published.back().velocity.x);

  EXPECT_TRUE(publisher.CoreSilent(milliseconds(3000)));
  ASSERT_EQ(3u, published.size());
  EXPECT_DOUBLE_EQ(0.0, published.back().velocity.x);
  EXPECT_DOUBLE_EQ(0.0, published.back().yaw_rate);
  EXPECT_DOUBLE_EQ(1.0, published.back().position.z);
  // Released: nothing is published any more and a second call does nothing.
  EXPECT_FALSE(publisher.CoreSilent(milliseconds(4000)));
  for (int tick = 0; tick < 100; ++tick)
    publisher.PublishPeriodic();
  EXPECT_EQ(3u, published.size());
  // A new intent after Core returned works as before.
  ASSERT_TRUE(publisher.SetIntent(1, -1, 0, 0, &error)) << error;
  EXPECT_DOUBLE_EQ(-0.5, published.back().velocity.x);
}

TEST(RemoteControlPublisher, CoreSilenceNeverActsOnAnInactiveIntentOrWithTheLimitOff) {
  ros::Time::init();
  std::vector<mavros_msgs::PositionTarget> published;
  auto make = [&published](milliseconds limit) {
    return std::make_unique<RemoteControlPublisher>(
        [&published](const mavros_msgs::PositionTarget &target) {
          published.push_back(target);
        },
        1.0, 1.5, 0.9, limit);
  };
  std::string error;
  auto armed_limit = make(milliseconds(3000));
  EXPECT_FALSE(armed_limit->CoreSilent(milliseconds(60000)));  // nothing active
  EXPECT_TRUE(published.empty());

  auto off = make(milliseconds(0));
  ASSERT_TRUE(off->SetIntent(3, 1, 0, 0, &error)) << error;
  published.clear();
  EXPECT_FALSE(off->CoreSilent(milliseconds(600000)));
  off->PublishPeriodic();
  ASSERT_EQ(1u, published.size());
  EXPECT_DOUBLE_EQ(1.5, published.back().velocity.x);
}

TEST(RemoteControlPublisher, FailedPublicationLeavesTheIntentForTheNextTick) {
  ros::Time::init();
  int attempts = 0;
  std::vector<mavros_msgs::PositionTarget> published;
  RemoteControlPublisher publisher(
      [&](const mavros_msgs::PositionTarget &target) {
        if (attempts++ == 1)
          throw std::runtime_error("transport down");
        published.push_back(target);
      },
      1.0, 1.5, 0.9, milliseconds(3000));
  std::string error;
  ASSERT_TRUE(publisher.SetIntent(3, 1, 0, 0, &error)) << error;
  EXPECT_FALSE(publisher.CoreSilent(milliseconds(3000)));  // publish throws
  EXPECT_TRUE(publisher.CoreSilent(milliseconds(3100)));
  EXPECT_DOUBLE_EQ(0.0, published.back().velocity.x);
}

TEST(RemoteControlPublisher, RejectsOutOfRangeAxesAndGear) {
  RemoteControlPublisher publisher(
      [](const mavros_msgs::PositionTarget &) {},1.0,1.5,1.0);
  std::string error;
  EXPECT_FALSE(publisher.SetIntent(0,0,0,0,&error));
  EXPECT_FALSE(publisher.SetIntent(4,0,0,0,&error));
  EXPECT_FALSE(publisher.SetIntent(1,2,0,0,&error));
  EXPECT_FALSE(publisher.SetIntent(1,0,-2,0,&error));
  EXPECT_FALSE(publisher.SetIntent(1,0,0,2,&error));
}

} // namespace
} // namespace xgc_px4_multirotor_ros1_adapter
