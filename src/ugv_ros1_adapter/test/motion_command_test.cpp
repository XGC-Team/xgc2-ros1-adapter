#include <gtest/gtest.h>

#include <chrono>
#include <stdexcept>
#include <vector>

#include "xgc_ugv_ros1_adapter/motion_command.hpp"

namespace xgc_ugv_ros1_adapter {
namespace {

using std::chrono::milliseconds;

const MotionLimits kLimits{1.5, 1.0, false};

struct Wire {
  std::vector<geometry_msgs::Twist> published;
  MotionCommandPublisher::PublishFunction function() {
    return [this](const geometry_msgs::Twist &command) {
      if (fail)
        throw std::runtime_error("transport down");
      published.push_back(command);
    };
  }
  bool fail = false;
};

// Dead-man: Core has been silent for the limit; the stream ends with one zero
// Twist and does not restart by itself.
TEST(MotionCommandPublisher, CoreSilenceReleasesAnActiveIntentAfterTheLimit) {
  Wire wire;
  MotionCommandPublisher publisher(wire.function(), kLimits, milliseconds(3000));
  std::string error;
  ASSERT_TRUE(publisher.SetIntent(3, 1, 0, 0, &error)) << error;
  EXPECT_DOUBLE_EQ(1.5, wire.published.back().linear.x);

  EXPECT_FALSE(publisher.CoreSilent(milliseconds(2999)));
  publisher.PublishPeriodic();
  ASSERT_EQ(2u, wire.published.size());
  EXPECT_DOUBLE_EQ(1.5, wire.published.back().linear.x);

  EXPECT_TRUE(publisher.CoreSilent(milliseconds(3000)));
  ASSERT_EQ(3u, wire.published.size());
  EXPECT_DOUBLE_EQ(0.0, wire.published.back().linear.x);
  EXPECT_DOUBLE_EQ(0.0, wire.published.back().angular.z);
  // Released: the 10 Hz stream is over and a second call does nothing.
  EXPECT_FALSE(publisher.CoreSilent(milliseconds(4000)));
  for (int tick = 0; tick < 100; ++tick)
    publisher.PublishPeriodic();
  EXPECT_EQ(3u, wire.published.size());
  // A new intent after Core returned works as before.
  ASSERT_TRUE(publisher.SetIntent(1, 1, 0, 0, &error)) << error;
  EXPECT_NEAR(0.5, wire.published.back().linear.x, 1e-9);
  publisher.PublishPeriodic();
  EXPECT_EQ(5u, wire.published.size());
}

TEST(MotionCommandPublisher, CoreSilenceNeverActsOnAnInactiveIntentOrWithTheLimitOff) {
  Wire wire;
  MotionCommandPublisher armed(wire.function(), kLimits, milliseconds(3000));
  EXPECT_FALSE(armed.CoreSilent(milliseconds(60000)));  // nothing active
  EXPECT_TRUE(wire.published.empty());

  MotionCommandPublisher off(wire.function(), kLimits, milliseconds(0));
  std::string error;
  ASSERT_TRUE(off.SetIntent(3, 1, 0, 0, &error)) << error;
  wire.published.clear();
  EXPECT_FALSE(off.CoreSilent(milliseconds(600000)));
  off.PublishPeriodic();
  ASSERT_EQ(1u, wire.published.size());
  EXPECT_DOUBLE_EQ(1.5, wire.published.back().linear.x);

  // After an operator release the dead-man has nothing to do either.
  MotionCommandPublisher released(wire.function(), kLimits, milliseconds(3000));
  ASSERT_TRUE(released.SetIntent(3, 1, 0, 0, &error)) << error;
  ASSERT_TRUE(released.Release(&error)) << error;
  wire.published.clear();
  EXPECT_FALSE(released.CoreSilent(milliseconds(5000)));
  EXPECT_TRUE(wire.published.empty());
}

TEST(MotionCommandPublisher, FailedPublicationLeavesTheIntentForTheNextTick) {
  Wire wire;
  MotionCommandPublisher publisher(wire.function(), kLimits, milliseconds(3000));
  std::string error;
  ASSERT_TRUE(publisher.SetIntent(3, 1, 0, 0, &error)) << error;
  wire.fail = true;
  EXPECT_FALSE(publisher.CoreSilent(milliseconds(3000)));
  wire.fail = false;
  EXPECT_TRUE(publisher.CoreSilent(milliseconds(3100)));
  EXPECT_DOUBLE_EQ(0.0, wire.published.back().linear.x);
}

} // namespace
} // namespace xgc_ugv_ros1_adapter

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::Time::init();
  return RUN_ALL_TESTS();
}
