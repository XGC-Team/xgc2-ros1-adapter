#include <gtest/gtest.h>

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
