#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "xgc/robot/v1/message.pb.h"
#include "xgc/semantic/common/v1/telemetry.pb.h"
#include "xgc_mocap_rotor_ros1_adapter/generated_contract.hpp"
#include "xgc_mocap_rotor_ros1_adapter/robot_runtime.hpp"

namespace xgc_mocap_rotor_ros1_adapter {
namespace {

void ensureRosInitialized() {
  if (ros::isInitialized()) {
    return;
  }
  int argc = 1;
  char name[] = "mocap_rotor_runtime_test";
  char *argv[] = {name, nullptr};
  ros::init(argc, argv, name, ros::init_options::NoSigintHandler);
}

xgc2_ros1_robot_adapter::RobotConfig robotConfig() {
  xgc2_ros1_robot_adapter::RobotConfig config;
  config.robot_id = "mocap-rotor-01";
  config.profile_id = contract::kProfileId;
  config.profile_digest = contract::profileDigest(contract::kProfileId);
  config.parameters = {
      {"namespace", "/mocap_rotor1"},
      {"robot_id", config.robot_id},
      {"wire_transport", "zenoh"},
      {"zenoh_listen", "tcp/0.0.0.0:7457"},
  };
  for (const std::string &channel :
       {"state.pose", "state.velocity", "state.speed", "state.imu",
        "state.power", "state.health", "state.flight", "diagnostic.link",
        "diagnostic.stream-health"}) {
    config.channels.push_back({channel, true});
  }
  return config;
}

std::vector<std::string> channelIds(const std::vector<std::string> &items) {
  std::vector<std::string> channels;
  for (const auto &item : items) {
    xgc::robot::v1::RobotMessage message;
    EXPECT_TRUE(message.ParseFromString(item));
    channels.push_back(message.channel_id());
  }
  return channels;
}

bool latestPoseEstimate(const std::vector<std::string> &items,
                        xgc::semantic::common::v1::PoseEstimate *pose) {
  if (pose == nullptr)
    return false;
  for (auto item = items.rbegin(); item != items.rend(); ++item) {
    xgc::robot::v1::RobotMessage message;
    if (!message.ParseFromString(*item) || message.channel_id() != "state.pose")
      continue;
    return pose->ParseFromString(message.message().payload().value());
  }
  return false;
}

struct PoseSurfaces {
  std::mutex mutex;
  std::condition_variable changed;
  bool have_pose = false;
  bool have_odometry = false;
  bool have_path = false;
  geometry_msgs::PoseStamped pose;
  nav_msgs::Odometry odometry;
  nav_msgs::Path path;
};

bool waitForPublishers(const ros::Subscriber &pose,
                       const ros::Subscriber &odometry,
                       const ros::Subscriber &path) {
  const ros::WallTime deadline = ros::WallTime::now() + ros::WallDuration(2.0);
  while (ros::ok() && ros::WallTime::now() < deadline) {
    if (pose.getNumPublishers() == 1u && odometry.getNumPublishers() == 1u &&
        path.getNumPublishers() == 1u) {
      return true;
    }
    ros::WallDuration(0.01).sleep();
  }
  return false;
}

bool waitForPoseSurfaces(PoseSurfaces *surfaces) {
  if (surfaces == nullptr)
    return false;
  std::unique_lock<std::mutex> lock(surfaces->mutex);
  return surfaces->changed.wait_for(lock, std::chrono::seconds(2), [surfaces] {
    return surfaces->have_pose && surfaces->have_odometry &&
           surfaces->have_path;
  });
}

std::shared_ptr<RobotRuntime> createRuntime(std::vector<std::string> *emitted,
                                            std::string *error) {
  return RobotRuntime::Create(
      ros::NodeHandle(), robotConfig(), 7u,
      [emitted](std::string item) {
        if (emitted != nullptr)
          emitted->push_back(std::move(item));
      },
      error);
}

std::string poseWireFrame(std::uint64_t sequence, const std::string &parent,
                          const std::string &child) {
  return std::string(R"({
    "v":1,"sequence":)") +
         std::to_string(sequence) + R"(,"t_ms":1700000000000,
    "frame_id":")" +
         parent + R"(","child_frame_id":")" + child + R"(",
    "position":{"x":1.0,"y":2.0,"z":3.0},
    "orientation":{"x":0.0,"y":0.0,"z":0.0,"w":1.0}
  })";
}

TEST(MocapRotorRuntime, NormalizesMavrosMapAcrossEveryGroundPoseSurface) {
  ensureRosInitialized();
  ros::AsyncSpinner spinner(2u);
  spinner.start();

  std::vector<std::string> emitted;
  std::string error;
  auto runtime = createRuntime(&emitted, &error);
  ASSERT_TRUE(runtime) << error;

  PoseSurfaces surfaces;
  ros::NodeHandle node;
  const auto pose_subscriber = node.subscribe<geometry_msgs::PoseStamped>(
      "/mocap_rotor1/local_pose", 1,
      [&surfaces](const geometry_msgs::PoseStamped::ConstPtr &message) {
        std::lock_guard<std::mutex> lock(surfaces.mutex);
        surfaces.pose = *message;
        surfaces.have_pose = true;
        surfaces.changed.notify_all();
      });
  const auto odometry_subscriber = node.subscribe<nav_msgs::Odometry>(
      "/mocap_rotor1/odom", 1,
      [&surfaces](const nav_msgs::Odometry::ConstPtr &message) {
        std::lock_guard<std::mutex> lock(surfaces.mutex);
        surfaces.odometry = *message;
        surfaces.have_odometry = true;
        surfaces.changed.notify_all();
      });
  const auto path_subscriber = node.subscribe<nav_msgs::Path>(
      "/mocap_rotor1/path", 1,
      [&surfaces](const nav_msgs::Path::ConstPtr &message) {
        std::lock_guard<std::mutex> lock(surfaces.mutex);
        surfaces.path = *message;
        surfaces.have_path = true;
        surfaces.changed.notify_all();
      });
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  ASSERT_TRUE(
      waitForPublishers(pose_subscriber, odometry_subscriber, path_subscriber));
  ASSERT_TRUE(runtime->HandleWireFrame(
      WireChannel::kLocalPose, poseWireFrame(1u, "map", "base_link"), &error))
      << error;
  ASSERT_TRUE(waitForPoseSurfaces(&surfaces));

  xgc::semantic::common::v1::PoseEstimate semantic;
  ASSERT_TRUE(latestPoseEstimate(emitted, &semantic));
  EXPECT_EQ(semantic.frame_id(), "world");
  EXPECT_EQ(semantic.child_frame_id(), "mocap_rotor1/base_link");

  {
    std::lock_guard<std::mutex> lock(surfaces.mutex);
    EXPECT_EQ(surfaces.pose.header.frame_id, "world");
    EXPECT_EQ(surfaces.odometry.header.frame_id, "world");
    EXPECT_EQ(surfaces.odometry.child_frame_id, "mocap_rotor1/base_link");
    ASSERT_EQ(surfaces.path.header.frame_id, "world");
    ASSERT_EQ(surfaces.path.poses.size(), 1u);
    EXPECT_EQ(surfaces.path.poses.front().header.frame_id, "world");
  }

  ASSERT_TRUE(tf_buffer.canTransform("world", "mocap_rotor1/base_link",
                                     ros::Time(0), ros::Duration(2.0)));
  const auto transform = tf_buffer.lookupTransform(
      "world", "mocap_rotor1/base_link", ros::Time(0));
  EXPECT_EQ(transform.header.frame_id, "world");
  EXPECT_EQ(transform.child_frame_id, "mocap_rotor1/base_link");
  runtime->Stop();
  spinner.stop();
}

TEST(MocapRotorRuntime, PreservesWorldAndPrefixesFramesOnlyOnce) {
  ensureRosInitialized();
  std::vector<std::string> emitted;
  std::string error;
  auto runtime = createRuntime(&emitted, &error);
  ASSERT_TRUE(runtime) << error;

  ASSERT_TRUE(runtime->HandleWireFrame(
      WireChannel::kLocalPose,
      poseWireFrame(1u, "world", "mocap_rotor1/base_link"), &error))
      << error;
  xgc::semantic::common::v1::PoseEstimate semantic;
  ASSERT_TRUE(latestPoseEstimate(emitted, &semantic));
  EXPECT_EQ(semantic.frame_id(), "world");
  EXPECT_EQ(semantic.child_frame_id(), "mocap_rotor1/base_link");
  EXPECT_EQ(semantic.child_frame_id().find("mocap_rotor1/mocap_rotor1/"),
            std::string::npos);
  runtime->Stop();
}

TEST(MocapRotorRuntime, KeepsExistingQualificationForOtherParentFrames) {
  ensureRosInitialized();
  std::vector<std::string> emitted;
  std::string error;
  auto runtime = createRuntime(&emitted, &error);
  ASSERT_TRUE(runtime) << error;

  ASSERT_TRUE(runtime->HandleWireFrame(
      WireChannel::kLocalPose, poseWireFrame(1u, "odom", "base_link"), &error))
      << error;
  xgc::semantic::common::v1::PoseEstimate semantic;
  ASSERT_TRUE(latestPoseEstimate(emitted, &semantic));
  EXPECT_EQ(semantic.frame_id(), "mocap_rotor1/odom");
  EXPECT_EQ(semantic.child_frame_id(), "mocap_rotor1/base_link");
  runtime->Stop();
}

TEST(MocapRotorRuntime, ProjectsBoundedReadOnlyWireWithoutMavros) {
  ensureRosInitialized();
  std::vector<std::string> emitted;
  std::string error;
  auto runtime = RobotRuntime::Create(
      ros::NodeHandle(), robotConfig(), 7u,
      [&emitted](std::string item) { emitted.push_back(std::move(item)); },
      &error);
  ASSERT_TRUE(runtime) << error;

  const std::string pose = R"({
    "v":1,"sequence":1,"t_ms":1700000000000,
    "frame_id":"world","child_frame_id":"base_link",
    "position":{"x":1.0,"y":2.0,"z":3.0},
    "orientation":{"x":0.0,"y":0.0,"z":0.0,"w":1.0}
  })";
  ASSERT_TRUE(runtime->HandleWireFrame(WireChannel::kLocalPose, pose, &error))
      << error;
  error.clear();
  EXPECT_FALSE(runtime->HandleWireFrame(WireChannel::kLocalPose, pose, &error));
  EXPECT_NE(error.find("duplicate or regressed"), std::string::npos);

  const std::string velocity = R"({
    "v":1,"sequence":1,"t_ms":1700000000001,"frame_id":"base_link",
    "linear":{"x":1.0,"y":0.0,"z":0.0},
    "angular":{"x":0.0,"y":0.0,"z":0.1}
  })";
  ASSERT_TRUE(
      runtime->HandleWireFrame(WireChannel::kLocalVelocity, velocity, &error))
      << error;
  const std::string imu = R"({
    "v":1,"sequence":1,"t_ms":1700000000002,"frame_id":"base_link",
    "orientation":{"x":0.0,"y":0.0,"z":0.0,"w":1.0},
    "angular_velocity":{"x":0.0,"y":0.0,"z":0.1},
    "linear_acceleration":{"x":0.0,"y":0.0,"z":9.81},
    "covariance":{
      "orientation":[0,0,0,0,0,0,0,0,0],
      "angular_velocity":[0,0,0,0,0,0,0,0,0],
      "linear_acceleration":[0,0,0,0,0,0,0,0,0]
    }
  })";
  ASSERT_TRUE(runtime->HandleWireFrame(WireChannel::kImu, imu, &error))
      << error;
  const std::string power = R"({
    "v":1,"sequence":1,"t_ms":1700000000003,
    "percentage":0.75,"voltage_v":23.4,"current_a":null,
    "temperature_c":null,"charging":false
  })";
  ASSERT_TRUE(runtime->HandleWireFrame(WireChannel::kPower, power, &error))
      << error;
  const std::string flight = R"({
    "v":1,"sequence":1,"t_ms":1700000000004,
    "connected":true,"armed":false,"guided":false,"manual_input":false,
    "mode":"POSCTL","system_status":4,"landed_state":1,"faults":[]
  })";
  ASSERT_TRUE(
      runtime->HandleWireFrame(WireChannel::kFlightState, flight, &error))
      << error;
  const std::string heartbeat = R"({
    "v":1,"sequence":1,"t_ms":1700000000005,
    "robot_id":"mocap-rotor-01","transport":"zenoh","uptime_ms":5000,
    "channels":[
      {"id":"local_pose","source_samples":10,"source_age_ms":5,"ready":true},
      {"id":"local_velocity","source_samples":9,"source_age_ms":7,"ready":true},
      {"id":"flight_state","source_samples":2,"source_age_ms":20,"ready":true},
      {"id":"extended_state","source_samples":2,"source_age_ms":19,"ready":true}
    ],
    "stats":{"publish_success":23,"publish_failure":0,"throttled":4,"rejected_source":0}
  })";
  ASSERT_TRUE(runtime->HandleWireFrame(WireChannel::kForwarderHeartbeat,
                                       heartbeat, &error))
      << error;

  const std::string restarted_heartbeat = R"({
    "v":1,"sequence":1,"t_ms":1700000001005,
    "robot_id":"mocap-rotor-01","transport":"zenoh","uptime_ms":100,
    "channels":[],
    "stats":{"publish_success":0,"publish_failure":0,"throttled":0,"rejected_source":0}
  })";
  error.clear();
  ASSERT_TRUE(runtime->HandleWireFrame(WireChannel::kForwarderHeartbeat,
                                       restarted_heartbeat, &error))
      << error;
  const std::string restarted_pose = R"({
    "v":1,"sequence":1,"t_ms":1700000001006,
    "frame_id":"world","child_frame_id":"base_link",
    "position":{"x":1.0,"y":2.0,"z":3.0},
    "orientation":{"x":0.0,"y":0.0,"z":0.0,"w":1.0}
  })";
  ASSERT_TRUE(
      runtime->HandleWireFrame(WireChannel::kLocalPose, restarted_pose, &error))
      << error;
  error.clear();
  EXPECT_FALSE(runtime->HandleWireFrame(WireChannel::kForwarderHeartbeat,
                                        restarted_heartbeat, &error));
  EXPECT_NE(error.find("duplicate or regressed"), std::string::npos);
  runtime->EmitPeriodic(ros::WallTime::now());

  const auto channels = channelIds(emitted);
  for (const std::string &required :
       {"state.pose", "state.velocity", "state.speed", "state.imu",
        "state.power", "state.flight", "state.health", "diagnostic.link",
        "diagnostic.stream-health"}) {
    EXPECT_NE(std::find(channels.begin(), channels.end(), required),
              channels.end())
        << required;
  }
  for (const auto &channel : channels) {
    EXPECT_EQ(channel.find("operation."), std::string::npos);
    EXPECT_EQ(channel.find("gps"), std::string::npos);
    EXPECT_EQ(channel.find("setpoint"), std::string::npos);
  }
  runtime->Stop();
}

TEST(MocapRotorRuntime, ResetsOnlyForAForwardMovingRestartHeartbeat) {
  EXPECT_TRUE(ShouldResetWireEpoch(7u, 1u, 1000, 2000, 9000, 100, 0.1, 3.0));
  EXPECT_TRUE(ShouldResetWireEpoch(7u, 7u, 1000, 2000, 9000, 12000, 3.1, 3.0));
  EXPECT_FALSE(ShouldResetWireEpoch(7u, 1u, 2000, 1000, 9000, 100, 5.0, 3.0));
  EXPECT_FALSE(ShouldResetWireEpoch(7u, 8u, 1000, 2000, 9000, 100, 5.0, 3.0));
  EXPECT_FALSE(ShouldResetWireEpoch(7u, 7u, 1000, 2000, 9000, 12000, 2.9, 3.0));
}

TEST(MocapRotorRuntime, RefusesFallbackTransportAndDisabledBaseline) {
  ensureRosInitialized();
  std::string error;
  auto fallback = robotConfig();
  fallback.parameters["wire_transport"] = "tcp";
  EXPECT_FALSE(RobotRuntime::Create(
      ros::NodeHandle(), fallback, 1u, [](std::string) {}, &error));
  EXPECT_NE(error.find("no fallback"), std::string::npos);

  auto partial = robotConfig();
  partial.channels.front().enabled = false;
  error.clear();
  EXPECT_FALSE(RobotRuntime::Create(
      ros::NodeHandle(), partial, 1u, [](std::string) {}, &error));
  EXPECT_NE(error.find("baseline channel is disabled"), std::string::npos);
}

} // namespace
} // namespace xgc_mocap_rotor_ros1_adapter

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
