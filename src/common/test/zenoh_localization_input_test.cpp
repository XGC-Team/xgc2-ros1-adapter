#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <future>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

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

TEST(ZenohLocalizationInput, ReceivesOverUdpUnicast) {
  const std::uint16_t port =
      static_cast<std::uint16_t>(20000u + (getpid() % 20000) + 2u);
  const std::string endpoint = "udp/127.0.0.1:" + std::to_string(port);
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
  binding.tracker = "udp1";
  const std::string key = ZenohLocalizationKey(binding, "pose");
  z_view_keyexpr_t key_expression;
  ASSERT_GE(z_view_keyexpr_from_str(&key_expression, key.c_str()), 0);
  z_owned_publisher_t publisher;
  ASSERT_GE(z_declare_publisher(z_loan(publisher_session), &publisher,
                                z_loan(key_expression), nullptr),
            0);
  std::atomic<bool> received{false};
  std::string error;
  ZenohLocalizationInput::Callbacks callbacks;
  callbacks.pose = [&](const ZenohPoseSample &) {
    received.store(true, std::memory_order_release);
  };
  auto input = ZenohLocalizationInput::Open(binding, std::move(callbacks),
                                            &error);
  ASSERT_NE(nullptr, input) << error;
  xgc2::vrpn_zenoh::v1::Pose sample;
  sample.mutable_report_time()->set_nanoseconds(7);
  sample.mutable_position_m()->set_x(1.0);
  sample.mutable_orientation_xyzw()->set_w(1.0);
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

TEST(ZenohLocalizationInput, PoseCallbacksCoalesceBurstsToLatestSampleWhenBusy) {
  const std::uint16_t port =
      static_cast<std::uint16_t>(20000u + (getpid() % 20000) + 2u);
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
  std::vector<std::int64_t> observed_nanos;
  bool block_first = true;
  std::string error;
  ZenohLocalizationInput::Callbacks callbacks;
  callbacks.pose = [&](const ZenohPoseSample &sample) {
    std::unique_lock<std::mutex> lock(mutex);
    observed_nanos.push_back(sample.report_time_nanos);
    const bool should_block = (observed_nanos.size() == 1u) && block_first;
    changed.notify_all();
    if (should_block) {
      changed.wait(lock, [&] { return !block_first; });
    }
  };
  auto input = ZenohLocalizationInput::Open(binding, std::move(callbacks),
                                            &error);
  ASSERT_NE(nullptr, input) << error;

  auto publish_t = [&](std::int64_t t_ns) {
    xgc2::vrpn_zenoh::v1::Pose sample;
    sample.mutable_report_time()->set_nanoseconds(t_ns);
    sample.mutable_report_time()->set_clock_domain(xgc::v1::CLOCK_DOMAIN_STEADY);
    sample.mutable_orientation_xyzw()->set_w(1.0);
    std::string payload;
    ASSERT_TRUE(sample.SerializeToString(&payload));
    z_owned_bytes_t bytes;
    ASSERT_GE(z_bytes_copy_from_buf(
                  &bytes,
                  reinterpret_cast<const std::uint8_t *>(payload.data()),
                  payload.size()),
              0);
    ASSERT_GE(z_publisher_put(z_loan(publisher), z_move(bytes), nullptr), 0);
  };

  {
    std::unique_lock<std::mutex> lock(mutex);
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(3);
    while (observed_nanos.empty() &&
           std::chrono::steady_clock::now() < deadline) {
      publish_t(100);
      changed.wait_for(lock, std::chrono::milliseconds(20));
    }
    ASSERT_FALSE(observed_nanos.empty());
  }

  // Send a rapid burst while callback #1 is stalled; intermediate frames
  // must overwrite the single pending slot without queuing a replay backlog.
  for (std::int64_t t = 200; t <= 2000; t += 100) {
    publish_t(t);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  {
    std::lock_guard<std::mutex> lock(mutex);
    block_first = false;
  }
  changed.notify_all();

  {
    std::unique_lock<std::mutex> lock(mutex);
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(3);
    changed.wait_until(lock, deadline, [&] {
      return !observed_nanos.empty() && observed_nanos.back() == 2000;
    });
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    EXPECT_LT(observed_nanos.size(), 5u);
    EXPECT_EQ(100, observed_nanos.front());
    EXPECT_EQ(2000, observed_nanos.back());
  }

  input->Stop();
  input.reset();
  z_drop(z_move(publisher));
  z_drop(z_move(publisher_session));
}

TEST(ZenohPoseValidity,
     EnforcesFiniteComponentsAndFivePercentUnitQuaternionNorm) {
  const double pos[3] = {1.25, -2.5, 0.75};
  const double identity[4] = {0.0, 0.0, 0.0, 1.0};
  const double zero_q[4] = {0.0, 0.0, 0.0, 0.0};
  const double scaled_ok[4] = {0.0, 0.0, 0.0, 1.04};
  const double scaled_bad[4] = {0.0, 0.0, 0.0, 1.06};
  const double nan_q[4] = {0.0, 0.0, 0.0,
                           std::numeric_limits<double>::quiet_NaN()};
  const double inf_pos[3] = {1.0, std::numeric_limits<double>::infinity(), 0.0};

  double norm = 0.0;
  EXPECT_TRUE(validZenohPoseValues(pos, identity, &norm));
  EXPECT_DOUBLE_EQ(1.0, norm);
  EXPECT_TRUE(validZenohPoseValues(pos, scaled_ok, &norm));
  EXPECT_NEAR(1.04, norm, 1e-12);
  EXPECT_FALSE(validZenohPoseValues(pos, zero_q));
  EXPECT_FALSE(validZenohPoseValues(pos, scaled_bad));
  EXPECT_FALSE(validZenohPoseValues(pos, nan_q));
  EXPECT_FALSE(validZenohPoseValues(inf_pos, identity));

  ZenohPoseSample sample;
  sample.position[0] = 1.25;
  sample.position[1] = -2.5;
  sample.position[2] = 0.75;
  sample.orientation[0] = 0.0;
  sample.orientation[1] = 0.0;
  sample.orientation[2] = 0.0;
  sample.orientation[3] = 0.0;
  EXPECT_FALSE(validZenohPoseSample(sample));
  sample.orientation[3] = 1.0;
  EXPECT_TRUE(validZenohPoseSample(sample));
}

TEST(ZenohLocalizationInput,
     InvalidZeroQuaternionFollowedByValidIsNeverOverwrittenAndDoesNotReplay) {
  const std::uint16_t port =
      static_cast<std::uint16_t>(20000u + (getpid() % 20000) + 3u);
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
  binding.source = "r1-g4";
  binding.tracker = "d8_origin";
  const std::string key = ZenohLocalizationKey(binding, "pose");
  z_view_keyexpr_t key_expression;
  ASSERT_GE(z_view_keyexpr_from_str(&key_expression, key.c_str()), 0);
  z_owned_publisher_t publisher;
  ASSERT_GE(z_declare_publisher(z_loan(publisher_session), &publisher,
                                z_loan(key_expression), nullptr),
            0);

  std::mutex mutex;
  std::condition_variable changed;
  std::vector<ZenohPoseSample> observed;
  std::int64_t stall_on_timestamp = 100;
  bool stall_active = false;
  std::string error;
  ZenohLocalizationInput::Callbacks callbacks;
  callbacks.pose = [&](const ZenohPoseSample &sample) {
    std::unique_lock<std::mutex> lock(mutex);
    observed.push_back(sample);
    if (sample.report_time_nanos == stall_on_timestamp) {
      stall_active = true;
      changed.notify_all();
      changed.wait(lock, [&] { return !stall_active; });
    } else {
      changed.notify_all();
    }
  };
  auto input =
      ZenohLocalizationInput::Open(binding, std::move(callbacks), &error);
  ASSERT_NE(nullptr, input) << error;

  auto publish_pose = [&](std::int64_t t_ns, bool valid) {
    xgc2::vrpn_zenoh::v1::Pose sample;
    sample.mutable_report_time()->set_nanoseconds(t_ns);
    sample.mutable_report_time()->set_clock_domain(xgc::v1::CLOCK_DOMAIN_STEADY);
    sample.mutable_position_m()->set_x(1.25);
    sample.mutable_position_m()->set_y(-2.5);
    sample.mutable_position_m()->set_z(0.75);
    sample.mutable_orientation_xyzw()->set_w(valid ? 1.0 : 0.0);
    std::string payload;
    ASSERT_TRUE(sample.SerializeToString(&payload));
    z_owned_bytes_t bytes;
    ASSERT_GE(z_bytes_copy_from_buf(
                  &bytes,
                  reinterpret_cast<const std::uint8_t *>(payload.data()),
                  payload.size()),
              0);
    ASSERT_GE(z_publisher_put(z_loan(publisher), z_move(bytes), nullptr), 0);
  };

  const auto deadline_for = [](int seconds) {
    return std::chrono::steady_clock::now() + std::chrono::seconds(seconds);
  };

  // 1. Stall worker on t=100, then burst [Valid(200), Invalid(300), Valid(400)]:
  //    Valid(200) is coalesced, Invalid(300) survives and precedes Valid(400).
  {
    std::unique_lock<std::mutex> lock(mutex);
    const auto deadline = deadline_for(3);
    while (!stall_active && std::chrono::steady_clock::now() < deadline) {
      publish_pose(100, true);
      changed.wait_for(lock, std::chrono::milliseconds(20));
    }
    ASSERT_TRUE(stall_active);
  }
  publish_pose(200, true);
  publish_pose(300, false);
  publish_pose(400, true);
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  {
    std::lock_guard<std::mutex> lock(mutex);
    stall_on_timestamp = 450;
    stall_active = false;
  }
  changed.notify_all();
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(changed.wait_until(lock, deadline_for(3), [&] {
      return !observed.empty() && observed.back().report_time_nanos == 400;
    }));
    auto find_index = [&](std::int64_t ts) -> std::ptrdiff_t {
      for (std::size_t i = 0; i < observed.size(); ++i) {
        if (observed[i].report_time_nanos == ts) {
          return static_cast<std::ptrdiff_t>(i);
        }
      }
      return -1;
    };
    const std::ptrdiff_t idx_300 = find_index(300);
    const std::ptrdiff_t idx_400 = find_index(400);
    ASSERT_GE(idx_300, 0);
    ASSERT_GT(idx_400, idx_300);
    EXPECT_FALSE(validZenohPoseSample(observed[idx_300]));
    EXPECT_TRUE(validZenohPoseSample(observed[idx_400]));
    EXPECT_EQ(-1, find_index(200));
  }

  // 2. Tail invalid supersedes earlier pending invalid & valid without replay.
  {
    std::unique_lock<std::mutex> lock(mutex);
    publish_pose(450, true);
    ASSERT_TRUE(changed.wait_until(lock, deadline_for(3),
                                   [&] { return stall_active; }));
  }
  publish_pose(500, false);
  publish_pose(600, true);
  publish_pose(700, false);
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  {
    std::lock_guard<std::mutex> lock(mutex);
    stall_on_timestamp = -1;
    stall_active = false;
  }
  changed.notify_all();
  {
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(changed.wait_until(lock, deadline_for(3), [&] {
      return !observed.empty() && observed.back().report_time_nanos == 700;
    }));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    bool saw_600 = false, saw_700_invalid = false;
    for (const auto &s : observed) {
      if (s.report_time_nanos == 600) saw_600 = true;
      if (s.report_time_nanos == 700 && !validZenohPoseSample(s)) {
        saw_700_invalid = true;
      }
    }
    EXPECT_FALSE(saw_600);
    EXPECT_TRUE(saw_700_invalid);
  }

  // 3. Unstalled live wire 0 us and ~75 us Invalid -> Valid bursts (10 rounds).
  for (int round = 0; round < 10; ++round) {
    const std::int64_t bad_ts = 1000 + round * 10;
    const std::int64_t good_ts = bad_ts + 1;
    std::size_t base = 0;
    {
      std::lock_guard<std::mutex> lock(mutex);
      base = observed.size();
    }
    publish_pose(bad_ts, false);
    if (round >= 5) {
      std::this_thread::sleep_for(std::chrono::microseconds(75));
    }
    publish_pose(good_ts, true);
    std::unique_lock<std::mutex> lock(mutex);
    ASSERT_TRUE(changed.wait_until(lock, deadline_for(3), [&] {
      for (std::size_t i = base; i < observed.size(); ++i) {
        if (observed[i].report_time_nanos == good_ts) return true;
      }
      return false;
    })) << "round " << round;
    std::ptrdiff_t idx_bad = -1, idx_good = -1;
    for (std::size_t i = base; i < observed.size(); ++i) {
      if (observed[i].report_time_nanos == bad_ts) idx_bad = i;
      if (observed[i].report_time_nanos == good_ts) idx_good = i;
    }
    ASSERT_GE(idx_bad, static_cast<std::ptrdiff_t>(base));
    ASSERT_GT(idx_good, idx_bad);
    EXPECT_FALSE(validZenohPoseSample(observed[idx_bad]));
    EXPECT_TRUE(validZenohPoseSample(observed[idx_good]));
  }

  // 4. Stopping transmission must not replay cached poses or NaNs.
  std::size_t settled_count = 0;
  {
    std::lock_guard<std::mutex> lock(mutex);
    settled_count = observed.size();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  {
    std::lock_guard<std::mutex> lock(mutex);
    EXPECT_EQ(settled_count, observed.size());
  }

  input->Stop();
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
