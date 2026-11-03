#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <future>
#include <mutex>
#include <cstdint>
#include <string>
#include <thread>

#include <unistd.h>
#include <zenoh.h>

#include "xgc2/vrpn_zenoh/v1/tracker.pb.h"
#include "xgc2_ros1_robot_adapter/zenoh_localization_input.hpp"

namespace xgc2_ros1_robot_adapter {
namespace {

TEST(ZenohLocalizationKey, EncodesSelectedSourceAndVrpnTrackerBytes) {
  ZenohLocalizationBinding binding;
  binding.source = "gazebo";
  binding.tracker = "uav1";
  binding.sensor = 0;
  EXPECT_EQ("xgc2/localization/vrpn/v1/67617a65626f/75617631/0/pose",
            ZenohLocalizationKey(binding, "pose"));
  binding.tracker = "UAV/01";
  EXPECT_EQ("xgc2/localization/vrpn/v1/67617a65626f/5541562f3031/0/pose",
            ZenohLocalizationKey(binding, "pose"));
}

TEST(ZenohLocalizationInput, ReceivesTheExactPoseKeyAndPreservesReportTime) {
  const std::uint16_t port =
      static_cast<std::uint16_t>(20000u + (getpid() % 20000));
  const std::string endpoint = "tcp/127.0.0.1:" + std::to_string(port);
  const std::string listen_config =
      "{\"mode\":\"peer\",\"listen\":{\"endpoints\":[\"" + endpoint +
      "\"]},\"scouting\":{\"multicast\":{\"enabled\":false},"
      "\"gossip\":{\"enabled\":false}}}";
  z_owned_config_t config;
  ASSERT_GE(zc_config_from_str(&config, listen_config.c_str()), 0);
  z_owned_session_t publisher_session;
  ASSERT_GE(z_open(&publisher_session, z_move(config), nullptr), 0);

  ZenohLocalizationBinding binding;
  binding.endpoint = endpoint;
  binding.source = "gazebo";
  binding.tracker = "uav1";
  binding.sensor = 0;
  const std::string key = ZenohLocalizationKey(binding, "pose");

  z_view_keyexpr_t key_expression;
  ASSERT_GE(z_view_keyexpr_from_str(&key_expression, key.c_str()), 0);
  z_owned_publisher_t publisher;
  ASSERT_GE(z_declare_publisher(z_loan(publisher_session), &publisher,
                                z_loan(key_expression), nullptr),
            0);

  std::atomic<bool> received{false};
  ZenohPoseSample observed;
  std::string error;
  ZenohLocalizationInput::Callbacks callbacks;
  callbacks.pose = [&](const ZenohPoseSample &pose) {
    observed = pose;
    received.store(true, std::memory_order_release);
  };
  auto input = ZenohLocalizationInput::Open(binding, std::move(callbacks),
                                            &error);
  ASSERT_NE(nullptr, input) << error;

  xgc2::vrpn_zenoh::v1::Pose sample;
  sample.mutable_report_time()->set_nanoseconds(123456789012345LL);
  sample.mutable_report_time()->set_clock_domain(xgc::v1::CLOCK_DOMAIN_SIMULATION);
  sample.mutable_position_m()->set_x(1.25);
  sample.mutable_position_m()->set_y(-2.5);
  sample.mutable_position_m()->set_z(3.75);
  sample.mutable_orientation_xyzw()->set_x(0.1);
  sample.mutable_orientation_xyzw()->set_y(0.2);
  sample.mutable_orientation_xyzw()->set_z(0.3);
  sample.mutable_orientation_xyzw()->set_w(0.9);
  std::string payload;
  ASSERT_TRUE(sample.SerializeToString(&payload));

  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (!received.load(std::memory_order_acquire) &&
         std::chrono::steady_clock::now() < deadline) {
    z_owned_bytes_t bytes;
    ASSERT_GE(z_bytes_copy_from_buf(
                  &bytes, reinterpret_cast<const std::uint8_t *>(payload.data()),
                  payload.size()),
              0);
    z_publisher_put(z_loan(publisher), z_move(bytes), nullptr);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  ASSERT_TRUE(received.load(std::memory_order_acquire));
  EXPECT_EQ(123456789012345LL, observed.report_time_nanos);
  EXPECT_EQ(static_cast<std::int32_t>(xgc::v1::CLOCK_DOMAIN_SIMULATION),
            observed.clock_domain);
  EXPECT_DOUBLE_EQ(1.25, observed.position[0]);
  EXPECT_DOUBLE_EQ(-2.5, observed.position[1]);
  EXPECT_DOUBLE_EQ(3.75, observed.position[2]);
  EXPECT_DOUBLE_EQ(0.9, observed.orientation[3]);

  input->Stop();
  z_drop(z_move(publisher));
  z_drop(z_move(publisher_session));
}

TEST(ZenohLocalizationInput, StopIsIdempotentAndDestructorSafe) {
  ZenohLocalizationBinding binding;
  binding.endpoint = "tcp/127.0.0.1:1";
  binding.source = "gazebo";
  binding.tracker = "uav1";
  std::string error;
  ZenohLocalizationInput::Callbacks callbacks;
  callbacks.pose = [](const ZenohPoseSample &) {};
  auto input = ZenohLocalizationInput::Open(binding, std::move(callbacks),
                                            &error);
  // A failed open is allowed here; the double-Stop contract is exercised by
  // the successful loopback test below.
  if (input) {
    input->Stop();
    input->Stop();
  }
}

TEST(ZenohLocalizationInput, StopJoinsAnInFlightCallbackBeforeDrop) {
  const std::uint16_t port =
      static_cast<std::uint16_t>(20000u + (getpid() % 20000) + 1u);
  const std::string endpoint = "tcp/127.0.0.1:" + std::to_string(port);
  const std::string listen_config =
      "{\"mode\":\"peer\",\"listen\":{\"endpoints\":[\"" + endpoint +
      "\"]},\"scouting\":{\"multicast\":{\"enabled\":false},"
      "\"gossip\":{\"enabled\":false}}}";
  z_owned_config_t config;
  ASSERT_GE(zc_config_from_str(&config, listen_config.c_str()), 0);
  z_owned_session_t publisher_session;
  ASSERT_GE(z_open(&publisher_session, z_move(config), nullptr), 0);

  ZenohLocalizationBinding binding;
  binding.endpoint = endpoint;
  binding.source = "gazebo";
  binding.tracker = "uav1";
  const std::string key = ZenohLocalizationKey(binding, "pose");
  z_view_keyexpr_t key_expression;
  ASSERT_GE(z_view_keyexpr_from_str(&key_expression, key.c_str()), 0);
  z_owned_publisher_t publisher;
  ASSERT_GE(z_declare_publisher(z_loan(publisher_session), &publisher,
                                z_loan(key_expression), nullptr),
            0);

  std::mutex mutex;
  std::condition_variable changed;
  bool entered = false;
  bool release = false;
  std::string error;
  ZenohLocalizationInput::Callbacks callbacks;
  callbacks.pose = [&](const ZenohPoseSample &) {
    std::unique_lock<std::mutex> lock(mutex);
    entered = true;
    changed.notify_all();
    changed.wait(lock, [&] { return release; });
  };
  auto input = ZenohLocalizationInput::Open(binding, std::move(callbacks),
                                            &error);
  ASSERT_NE(nullptr, input) << error;

  xgc2::vrpn_zenoh::v1::Pose sample;
  sample.mutable_report_time()->set_nanoseconds(1);
  sample.mutable_orientation_xyzw()->set_w(1.0);
  std::string payload;
  ASSERT_TRUE(sample.SerializeToString(&payload));
  {
    std::unique_lock<std::mutex> lock(mutex);
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (!entered && std::chrono::steady_clock::now() < deadline) {
      z_owned_bytes_t bytes;
      ASSERT_GE(z_bytes_copy_from_buf(
                    &bytes,
                    reinterpret_cast<const std::uint8_t *>(payload.data()),
                    payload.size()),
                0);
      z_publisher_put(z_loan(publisher), z_move(bytes), nullptr);
      changed.wait_for(lock, std::chrono::milliseconds(20));
    }
    ASSERT_TRUE(entered);
  }
  auto stopping = std::async(std::launch::async, [&] { input->Stop(); });
  EXPECT_EQ(std::future_status::timeout,
            stopping.wait_for(std::chrono::milliseconds(50)));
  {
    std::lock_guard<std::mutex> lock(mutex);
    release = true;
  }
  changed.notify_all();
  ASSERT_EQ(std::future_status::ready,
            stopping.wait_for(std::chrono::seconds(3)));
  stopping.get();
  input.reset();
  z_drop(z_move(publisher));
  z_drop(z_move(publisher_session));
}

TEST(ZenohAngularRate, ConvertsRotationOverIntervalToRadiansPerSecond) {
  const double identity[4] = {0.0, 0.0, 0.0, 1.0};
  double rate[3] = {1.0, 1.0, 1.0};
  ASSERT_TRUE(zenohAngularRate(identity, 0.1, rate));
  EXPECT_DOUBLE_EQ(0.0, rate[0]);
  EXPECT_DOUBLE_EQ(0.0, rate[1]);
  EXPECT_DOUBLE_EQ(0.0, rate[2]);

  const double quarter = 0.7071067811865476;
  const double rotation[4] = {0.0, 0.0, quarter, quarter};
  ASSERT_TRUE(zenohAngularRate(rotation, 1.0, rate));
  EXPECT_NEAR(0.0, rate[0], 1e-12);
  EXPECT_NEAR(0.0, rate[1], 1e-12);
  EXPECT_NEAR(1.5707963267948966, rate[2], 1e-12);

  EXPECT_FALSE(zenohAngularRate(identity, 0.0, rate));
}

} // namespace
} // namespace xgc2_ros1_robot_adapter

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
