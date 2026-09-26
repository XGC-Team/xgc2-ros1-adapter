#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <thread>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <zenoh.h>

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
#include "xgc_mocap_rotor_ros1_adapter/wire_contract.hpp"
#include "xgc_mocap_rotor_ros1_adapter/zenoh_subscriber.hpp"

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
        "state.power", "state.health", "state.flight", "state.controller",
        "diagnostic.link", "diagnostic.stream-health"}) {
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
  bool saw_imu = false;
  for (const auto &item : emitted) {
    xgc::robot::v1::RobotMessage message;
    ASSERT_TRUE(message.ParseFromString(item));
    if (message.channel_id() != "state.imu")
      continue;
    xgc::semantic::common::v1::ImuEstimate imu;
    ASSERT_TRUE(imu.ParseFromString(message.message().payload().value()));
    EXPECT_EQ(imu.orientation_covariance_size(), 0);
    EXPECT_EQ(imu.angular_velocity_covariance_size(), 0);
    EXPECT_EQ(imu.linear_acceleration_covariance_size(), 0);
    saw_imu = true;
  }
  EXPECT_TRUE(saw_imu);
  runtime->Stop();
}

TEST(MocapRotorRuntime, AbsentAngularAndChargingAreNotMeasured) {
  ensureRosInitialized();
  std::vector<std::string> emitted;
  std::string error;
  auto runtime = createRuntime(&emitted, &error);
  ASSERT_TRUE(runtime) << error;
  const std::string velocity = R"({
    "v":1,"sequence":1,"t_ms":1800000000000,"frame_id":"base_link",
    "linear":{"x":0.5,"y":0.0,"z":0.0},"angular":null
  })";
  ASSERT_TRUE(runtime->HandleWireFrame(WireChannel::kLocalVelocity, velocity, &error))
      << error;
  const std::string power = R"({
    "v":1,"sequence":1,"t_ms":1800000000001,
    "percentage":0.4,"voltage_v":15.5,"current_a":null,
    "temperature_c":null,"charging":null
  })";
  ASSERT_TRUE(runtime->HandleWireFrame(WireChannel::kPower, power, &error)) << error;
  bool saw_velocity = false;
  bool saw_power = false;
  for (const auto &item : emitted) {
    xgc::robot::v1::RobotMessage message;
    ASSERT_TRUE(message.ParseFromString(item));
    if (message.channel_id() == "state.velocity") {
      xgc::semantic::common::v1::VelocityEstimate estimate;
      ASSERT_TRUE(estimate.ParseFromString(message.message().payload().value()));
      EXPECT_FALSE(estimate.has_angular());
      EXPECT_DOUBLE_EQ(estimate.linear().x(), 0.5);
      saw_velocity = true;
    } else if (message.channel_id() == "state.power") {
      xgc::semantic::common::v1::PowerStatus status;
      ASSERT_TRUE(status.ParseFromString(message.message().payload().value()));
      EXPECT_DOUBLE_EQ(status.percentage(), 0.4);
      EXPECT_DOUBLE_EQ(status.voltage_v(), 15.5);
      saw_power = true;
    }
  }
  EXPECT_TRUE(saw_velocity);
  EXPECT_TRUE(saw_power);
  runtime->Stop();
}

TEST(MocapRotorRuntime, FlightSubstatesPassThroughControllerText) {
  ensureRosInitialized();
  for (const char *name : {"SelfCheck", "Hover", "Landing", "Takeoff"}) {
    std::vector<std::string> emitted;
    std::string error;
    auto runtime = createRuntime(&emitted, &error);
    ASSERT_TRUE(runtime) << error;
    const std::string heartbeat = std::string(R"({
      "v":1,"sequence":1,"t_ms":1800000000100,
      "robot_id":"mocap-rotor-01","transport":"zenoh","uptime_ms":40,
      "channels":[{"id":"controller","source_samples":1,"source_age_ms":5,"ready":true,"text":")") +
        name + R"("}],
      "stats":{"publish_success":1,"publish_failure":0,"throttled":0,"rejected_source":0}
    })";
    ASSERT_TRUE(runtime->HandleWireFrame(WireChannel::kForwarderHeartbeat, heartbeat, &error))
        << error;
    bool saw = false;
    for (const auto &item : emitted) {
      xgc::robot::v1::RobotMessage message;
      ASSERT_TRUE(message.ParseFromString(item));
      if (message.channel_id() != "state.controller")
        continue;
      xgc::semantic::common::v1::ControllerStatus status;
      ASSERT_TRUE(status.ParseFromString(message.message().payload().value()));
      EXPECT_EQ(status.text(), name);
      saw = true;
    }
    EXPECT_TRUE(saw) << name;
    runtime->Stop();
  }
  std::vector<std::string> emitted;
  std::string error;
  auto runtime = createRuntime(&emitted, &error);
  ASSERT_TRUE(runtime) << error;
  const std::string rejected = R"({
    "v":1,"sequence":1,"t_ms":1800000000101,
    "robot_id":"mocap-rotor-01","transport":"zenoh","uptime_ms":40,
    "channels":[{"id":"controller","source_samples":1,"source_age_ms":5,"ready":true,"text":"Hover\n"}],
    "stats":{"publish_success":1,"publish_failure":0,"throttled":0,"rejected_source":0}
  })";
  EXPECT_FALSE(runtime->HandleWireFrame(WireChannel::kForwarderHeartbeat, rejected, &error));
  EXPECT_NE(error.find("C0, DEL, or C1"), std::string::npos);
  runtime->Stop();

  const std::string forty_eight(48u, 'A');
  const std::string forty_nine(49u, 'A');
  const std::string c1_text = std::string("\xC2\x85");
  struct BoundCase {
    const char *label;
    std::string text;
    bool accepted;
  };
  const BoundCase bounds[] = {
      {"empty", "", false},
      {"c1", c1_text, false},
      {"48", forty_eight, true},
      {"49", forty_nine, false},
  };
  for (const auto &bound : bounds) {
    std::vector<std::string> bound_emitted;
    std::string bound_error;
    auto bound_runtime = createRuntime(&bound_emitted, &bound_error);
    ASSERT_TRUE(bound_runtime) << bound_error;
    const std::string heartbeat =
        std::string(R"({"v":1,"sequence":1,"t_ms":1800000000102,)") +
        R"("robot_id":"mocap-rotor-01","transport":"zenoh","uptime_ms":40,)" +
        R"("channels":[{"id":"controller","source_samples":1,"source_age_ms":5,"ready":true,"text":")" +
        bound.text + R"("}],)" +
        R"("stats":{"publish_success":1,"publish_failure":0,"throttled":0,"rejected_source":0}})";
    const bool ok = bound_runtime->HandleWireFrame(
        WireChannel::kForwarderHeartbeat, heartbeat, &bound_error);
    EXPECT_EQ(ok, bound.accepted) << bound.label << " " << bound_error;
    if (bound.accepted) {
      bool saw = false;
      for (const auto &item : bound_emitted) {
        xgc::robot::v1::RobotMessage message;
        ASSERT_TRUE(message.ParseFromString(item));
        if (message.channel_id() != "state.controller")
          continue;
        xgc::semantic::common::v1::ControllerStatus status;
        ASSERT_TRUE(status.ParseFromString(message.message().payload().value()));
        EXPECT_EQ(status.text(), bound.text);
        saw = true;
      }
      EXPECT_TRUE(saw) << bound.label;
    }
    bound_runtime->Stop();
  }
}

std::uint16_t unusedLoopbackPort() {
  const int socket_fd = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_fd < 0)
    throw std::runtime_error("cannot create loopback port probe socket");
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  address.sin_port = 0;
  if (bind(socket_fd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) !=
      0) {
    close(socket_fd);
    throw std::runtime_error("cannot bind loopback port probe socket");
  }
  socklen_t address_size = sizeof(address);
  if (getsockname(socket_fd, reinterpret_cast<sockaddr *>(&address),
                  &address_size) != 0) {
    close(socket_fd);
    throw std::runtime_error("cannot read loopback port probe socket");
  }
  close(socket_fd);
  return ntohs(address.sin_port);
}

TEST(MocapRotorRuntime, ZenohUplinkBecomesControllerPoseAndAbsentImuCovariance) {
  ensureRosInitialized();
  struct Shared {
    std::mutex mutex;
    std::condition_variable changed;
    std::vector<std::string> emitted;
    std::string error;
  };
  auto shared = std::make_shared<Shared>();
  std::string error;
  auto runtime = RobotRuntime::Create(
      ros::NodeHandle(), robotConfig(), 7u,
      [shared](std::string item) {
        std::lock_guard<std::mutex> lock(shared->mutex);
        shared->emitted.push_back(std::move(item));
        shared->changed.notify_all();
      },
      &error);
  ASSERT_TRUE(runtime) << error;

  const std::string endpoint =
      "tcp/127.0.0.1:" + std::to_string(unusedLoopbackPort());
  ZenohSubscriber subscriber(
      [runtime](std::string key, std::string payload) {
        ParsedWireKey parsed;
        std::string parse_error;
        if (!ParseWireKey(key, &parsed, &parse_error) ||
            parsed.robot_id != "mocap-rotor-01")
          return;
        std::string handle_error;
        runtime->HandleWireFrame(parsed.channel, payload, &handle_error);
      });
  ASSERT_TRUE(subscriber.Start(endpoint, &error)) << error;

  z_owned_config_t config;
  z_internal_null(&config);
  ASSERT_GE(z_config_default(&config), 0);
  const std::string connect = "['" + endpoint + "']";
  ASSERT_GE(zc_config_insert_json5(z_loan_mut(config), Z_CONFIG_MODE_KEY, "'peer'"),
            0);
  ASSERT_GE(
      zc_config_insert_json5(z_loan_mut(config), Z_CONFIG_CONNECT_KEY, connect.c_str()),
      0);
  ASSERT_GE(zc_config_insert_json5(z_loan_mut(config),
                                   Z_CONFIG_MULTICAST_SCOUTING_KEY, "false"),
            0);
  ASSERT_GE(zc_config_insert_json5(z_loan_mut(config), "scouting/gossip/enabled",
                                   "false"),
            0);
  z_owned_session_t publisher;
  z_internal_null(&publisher);
  ASSERT_GE(z_open(&publisher, z_move(config), nullptr), 0);

  const std::string pose = R"({
    "v":1,"sequence":1,"t_ms":2500,
    "frame_id":"world","child_frame_id":"base_link",
    "position":{"x":1.25,"y":2.5,"z":3.75},
    "orientation":{"x":0.0,"y":0.0,"z":0.0,"w":1.0}
  })";
  const std::string imu = R"({
    "v":1,"sequence":1,"t_ms":2501,"frame_id":"base_link",
    "orientation":null,
    "angular_velocity":{"x":0.0,"y":0.0,"z":0.25},
    "linear_acceleration":{"x":0.0,"y":0.0,"z":9.81},
    "covariance":{"orientation":null,"angular_velocity":null,"linear_acceleration":null}
  })";
  const std::string measured = R"({
    "v":1,"sequence":2,"t_ms":2700,"frame_id":"base_link",
    "orientation":{"x":0.0,"y":0.0,"z":0.0,"w":1.0},
    "angular_velocity":{"x":0.1,"y":0.0,"z":0.0},
    "linear_acceleration":{"x":0.0,"y":0.0,"z":9.7},
    "covariance":{
      "orientation":[0.2,0,0,0,0.2,0,0,0,0.2],
      "angular_velocity":[0.01,0,0,0,0.01,0,0,0,0.01],
      "linear_acceleration":[0.03,0,0,0,0.03,0,0,0,0.03]
    }
  })";
  const std::string heartbeat = R"({
    "v":1,"sequence":1,"t_ms":2502,
    "robot_id":"mocap-rotor-01","transport":"zenoh","uptime_ms":40,
    "channels":[
      {"id":"local_pose","source_samples":1,"source_age_ms":5,"ready":true},
      {"id":"controller","source_samples":1,"source_age_ms":5,"ready":true,"text":"Ready"}
    ],
    "stats":{"publish_success":1,"publish_failure":0,"throttled":0,"rejected_source":0}
  })";
  const std::vector<std::pair<std::string, std::string>> samples = {
      {"xgc2/mocap-rotor-01/up/local_pose", pose},
      {"xgc2/mocap-rotor-01/up/imu", imu},
      {"xgc2/mocap-rotor-01/up/forwarder_hb", heartbeat},
  };
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  bool saw_pose = false;
  bool saw_absent = false;
  bool saw_ready = false;
  while (std::chrono::steady_clock::now() < deadline &&
         !(saw_pose && saw_absent && saw_ready)) {
    for (const auto &sample : samples) {
      z_view_keyexpr_t key_expression;
      ASSERT_GE(z_view_keyexpr_from_str(&key_expression, sample.first.c_str()), 0);
      z_owned_bytes_t bytes;
      z_bytes_copy_from_buf(
          &bytes, reinterpret_cast<const uint8_t *>(sample.second.data()),
          sample.second.size());
      ASSERT_GE(z_put(z_loan(publisher), z_loan(key_expression), z_move(bytes), nullptr),
                0);
    }
    std::unique_lock<std::mutex> lock(shared->mutex);
    shared->changed.wait_for(lock, std::chrono::milliseconds(100));
    saw_pose = saw_absent = saw_ready = false;
    for (const auto &item : shared->emitted) {
      xgc::robot::v1::RobotMessage message;
      ASSERT_TRUE(message.ParseFromString(item));
      if (message.channel_id() == "state.pose") {
        xgc::semantic::common::v1::PoseEstimate pose_estimate;
        ASSERT_TRUE(
            pose_estimate.ParseFromString(message.message().payload().value()));
        saw_pose = pose_estimate.position().x() == 1.25;
      } else if (message.channel_id() == "state.imu") {
        xgc::semantic::common::v1::ImuEstimate imu_estimate;
        ASSERT_TRUE(
            imu_estimate.ParseFromString(message.message().payload().value()));
        if (!imu_estimate.has_orientation() &&
            imu_estimate.orientation_covariance_size() == 0 &&
            imu_estimate.angular_velocity().z() == 0.25)
          saw_absent = true;
      } else if (message.channel_id() == "state.controller") {
        xgc::semantic::common::v1::ControllerStatus status;
        ASSERT_TRUE(status.ParseFromString(message.message().payload().value()));
        saw_ready = status.text() == "Ready";
      }
    }
  }
  EXPECT_TRUE(saw_pose);
  EXPECT_TRUE(saw_absent);
  EXPECT_TRUE(saw_ready);

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  bool saw_measured = false;
  {
    z_view_keyexpr_t key_expression;
    ASSERT_GE(z_view_keyexpr_from_str(&key_expression, "xgc2/mocap-rotor-01/up/imu"),
              0);
    z_owned_bytes_t bytes;
    z_bytes_copy_from_buf(&bytes,
                          reinterpret_cast<const uint8_t *>(measured.data()),
                          measured.size());
    ASSERT_GE(
        z_put(z_loan(publisher), z_loan(key_expression), z_move(bytes), nullptr),
        0);
  }
  const auto measured_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (std::chrono::steady_clock::now() < measured_deadline && !saw_measured) {
    std::unique_lock<std::mutex> lock(shared->mutex);
    shared->changed.wait_for(lock, std::chrono::milliseconds(100));
    for (const auto &item : shared->emitted) {
      xgc::robot::v1::RobotMessage message;
      ASSERT_TRUE(message.ParseFromString(item));
      if (message.channel_id() != "state.imu")
        continue;
      xgc::semantic::common::v1::ImuEstimate imu_estimate;
      ASSERT_TRUE(
          imu_estimate.ParseFromString(message.message().payload().value()));
      if (imu_estimate.orientation_covariance_size() == 9 &&
          imu_estimate.orientation_covariance(0) == 0.2)
        saw_measured = true;
    }
  }
  EXPECT_TRUE(saw_measured);

  z_drop(z_move(publisher));
  subscriber.Stop();
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
