#include "xgc_ros1_tools_adapter/clock_wait.hpp"

#include <gtest/gtest.h>

namespace xgc_ros1_tools_adapter {

TEST(ClockWait, DecimalThresholdAndPausedSamples) {
  ClockWait wait("12.3");
  EXPECT_FALSE(wait.observe(12, 0));
  EXPECT_FALSE(wait.observe(12, 0));
  EXPECT_FALSE(wait.observe(12, 299999999));
  EXPECT_TRUE(wait.observe(12, 300000000));
}

TEST(ClockWait, JumpsZeroAndSubnanosecondThreshold) {
  ClockWait jump("12.5");
  EXPECT_FALSE(jump.observe(10, 0));
  EXPECT_TRUE(jump.observe(14, 0));
  ClockWait passed("12");
  EXPECT_TRUE(passed.observe(20, 0));
  ClockWait zero("0");
  EXPECT_FALSE(zero.observed());
  EXPECT_TRUE(zero.observe(0, 0));
  ClockWait tiny("0.0000000001");
  EXPECT_FALSE(tiny.observe(0, 0));
  EXPECT_TRUE(tiny.observe(0, 1));
}

TEST(ClockWait, ResetAndInvalidStampsFail) {
  ClockWait wait("12");
  EXPECT_FALSE(wait.observe(10, 0));
  EXPECT_THROW(wait.observe(9, 0), std::runtime_error);
  EXPECT_THROW(wait.observe(10, 1000000000), std::runtime_error);
}

TEST(ClockWait, ROSBoundsAndInvalidTargets) {
  ClockWait wait("4294967295");
  EXPECT_FALSE(wait.observe(UINT32_MAX - 1, 999999999));
  EXPECT_TRUE(wait.observe(UINT32_MAX, 0));
  for (const auto* value : {"", "-1", "NaN", "inf", "4294967296",
                            "4294967295.1", "1.", "1.2.3", "1e9"}) {
    EXPECT_THROW(clockTarget(value), std::invalid_argument) << value;
  }
}

}  // namespace xgc_ros1_tools_adapter
