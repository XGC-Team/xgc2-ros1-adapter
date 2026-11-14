#include <gtest/gtest.h>
#include <limits>
#include "xgc_px4_multirotor_ros1_adapter/position_error_window.hpp"

namespace xgc_px4_multirotor_ros1_adapter {
namespace {
geometry_msgs::PoseStamped sample(double time, double x,
                                  const std::string &frame = "world") {
  geometry_msgs::PoseStamped out;
  out.header.stamp.fromSec(time);
  out.header.frame_id = frame;
  out.pose.position.x = x;
  return out;
}

TEST(PositionErrorWindow, RemovesMotionDelayByInterpolation) {
  PositionErrorWindow window(0.05);
  window.local(sample(10.0, 0.0));
  window.local(sample(10.1, 0.1));
  ASSERT_TRUE(window.mocap(sample(10.1, 0.05)));
  ASSERT_TRUE(window.latest());
  EXPECT_NEAR(0.0, window.latest()->meters, 1e-12);
  EXPECT_NEAR(10.05, window.latest()->stamp.toSec(), 1e-12);
  EXPECT_EQ("world", window.latest()->frame);
}

TEST(PositionErrorWindow, WaitsForReferenceWithoutExtrapolation) {
  PositionErrorWindow window;
  window.local(sample(10.0, 1.0));
  EXPECT_FALSE(window.mocap(sample(10.05, 1.1)));
  EXPECT_FALSE(window.latest());
  ASSERT_TRUE(window.local(sample(10.1, 1.2)));
  EXPECT_NEAR(0.0, window.latest()->meters, 1e-12);
  EXPECT_FALSE(window.local(sample(10.12, 1.24)));
}

TEST(PositionErrorWindow, PreservesRealResidual) {
  PositionErrorWindow window(0.05);
  window.local(sample(10.0, 0.0));
  window.local(sample(10.1, 0.1));
  ASSERT_TRUE(window.mocap(sample(10.1, 0.08)));
  EXPECT_NEAR(0.03, window.latest()->meters, 1e-12);
}

TEST(PositionErrorWindow, RejectsMissingHistoryGapsAndDifferentFrames) {
  PositionErrorWindow window(0.05);
  window.local(sample(10.0, 0.0));
  EXPECT_FALSE(window.mocap(sample(10.01, 0.0)));
  window.local(sample(10.2, 0.2));
  EXPECT_FALSE(window.mocap(sample(10.2, 0.15)));
  EXPECT_FALSE(window.latest());
  EXPECT_FALSE(window.mocap(sample(10.25, 0.2, "other")));
  EXPECT_FALSE(window.latest());
}

TEST(PositionErrorWindow, ClockAndFrameChangesDiscardPreviousPair) {
  PositionErrorWindow window;
  window.local(sample(10.0, 2.0));
  ASSERT_TRUE(window.mocap(sample(10.0, 2.0)));
  window.local(sample(1.0, 5.0));
  EXPECT_FALSE(window.latest());
  ASSERT_TRUE(window.mocap(sample(1.0, 5.0)));
  window.local(sample(1.05, 8.0, "reset_world"));
  EXPECT_FALSE(window.latest());
  ASSERT_TRUE(window.mocap(sample(1.05, 8.0, "reset_world")));
  EXPECT_DOUBLE_EQ(0.0, window.latest()->meters);
}

TEST(PositionErrorWindow, InvalidInputAndZeroTimeDiscardPreviousPair) {
  PositionErrorWindow window;
  window.local(sample(10.0, 2.0));
  ASSERT_TRUE(window.mocap(sample(10.0, 2.0)));
  EXPECT_FALSE(window.mocap(sample(10.1, std::numeric_limits<double>::quiet_NaN())));
  EXPECT_FALSE(window.latest());
  window.local(sample(10.2, 2.0));
  ASSERT_TRUE(window.mocap(sample(10.2, 2.0)));
  EXPECT_FALSE(window.local(sample(0.0, 2.0)));
  EXPECT_FALSE(window.latest());
}

TEST(PositionErrorWindow, DoesNotEmitDuplicateMocapTimestamp) {
  PositionErrorWindow window;
  window.local(sample(10.0, 2.0));
  ASSERT_TRUE(window.mocap(sample(10.0, 2.0)));
  EXPECT_FALSE(window.mocap(sample(10.0, 3.0)));
  EXPECT_DOUBLE_EQ(0.0, window.latest()->meters);
}
} // namespace
} // namespace xgc_px4_multirotor_ros1_adapter

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
