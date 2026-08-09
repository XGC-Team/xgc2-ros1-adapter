#include <cmath>
#include <limits>
#include <string>

#include <gtest/gtest.h>
#include <nlohmann/json.hpp>

#include "xgc_mocap_rotor_zenoh_forwarder/wire_encoder.hpp"

namespace xgc_mocap_rotor_zenoh_forwarder {
namespace {

using Json = nlohmann::json;

TEST(MocapRotorWireEncoder, PreservesLocalPoseAndExplicitChildFrame) {
  geometry_msgs::PoseStamped message;
  message.header.frame_id = "map";
  message.pose.position.x = 1.0;
  message.pose.position.y = -2.0;
  message.pose.position.z = 3.0;
  message.pose.orientation.w = 1.0;
  std::string payload;
  std::string error;
  ASSERT_TRUE(EncodePose(message, "base_link", 7u, 1234, &payload, &error))
      << error;
  const Json body = Json::parse(payload);
  EXPECT_EQ(body.at("v"), 1);
  EXPECT_EQ(body.at("sequence"), 7u);
  EXPECT_EQ(body.at("t_ms"), 1234);
  EXPECT_EQ(body.at("frame_id"), "map");
  EXPECT_EQ(body.at("child_frame_id"), "base_link");
  EXPECT_DOUBLE_EQ(body.at("position").at("y"), -2.0);
  EXPECT_EQ(payload.find("gps"), std::string::npos);
  EXPECT_EQ(payload.find("cmd"), std::string::npos);
}

TEST(MocapRotorWireEncoder, CarriesAllImuCovariances) {
  sensor_msgs::Imu message;
  message.header.frame_id = "base_link";
  message.orientation.w = 1.0;
  message.angular_velocity.x = 0.1;
  message.linear_acceleration.z = 9.81;
  for (std::size_t index = 0u; index < 9u; ++index) {
    message.orientation_covariance[index] = static_cast<double>(index);
    message.angular_velocity_covariance[index] =
        static_cast<double>(index + 10u);
    message.linear_acceleration_covariance[index] =
        static_cast<double>(index + 20u);
  }
  std::string payload;
  std::string error;
  ASSERT_TRUE(EncodeImu(message, 1u, 2000, &payload, &error)) << error;
  const auto covariance = Json::parse(payload).at("covariance");
  EXPECT_EQ(covariance.at("orientation").size(), 9u);
  EXPECT_EQ(covariance.at("angular_velocity").size(), 9u);
  EXPECT_EQ(covariance.at("linear_acceleration").size(), 9u);
}

TEST(MocapRotorWireEncoder, MarksUnknownBatteryMeasurementsWithoutInventingValues) {
  sensor_msgs::BatteryState message;
  message.percentage = std::numeric_limits<float>::quiet_NaN();
  message.voltage = 23.4f;
  message.current = 4.5f;
  message.temperature = std::numeric_limits<float>::quiet_NaN();
  message.power_supply_status =
      sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
  std::string payload;
  std::string error;
  ASSERT_TRUE(EncodePower(message, 2u, 3000, &payload, &error)) << error;
  const Json body = Json::parse(payload);
  EXPECT_DOUBLE_EQ(body.at("percentage"), -1.0);
  EXPECT_TRUE(body.at("charging"));
  EXPECT_TRUE(body.at("temperature_c").is_null());
  EXPECT_FALSE(body.at("voltage_v").is_null());
}

TEST(MocapRotorWireEncoder, JoinsStateOnlyWithExtendedLandedState) {
  mavros_msgs::State state;
  state.connected = true;
  state.armed = false;
  state.guided = true;
  state.manual_input = false;
  state.mode = "POSCTL";
  state.system_status = 4u;
  mavros_msgs::ExtendedState extended;
  extended.landed_state = mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND;
  std::string payload;
  std::string error;
  ASSERT_TRUE(
      EncodeFlightState(state, extended, 5u, 4000, &payload, &error))
      << error;
  const Json body = Json::parse(payload);
  EXPECT_TRUE(body.at("connected"));
  EXPECT_EQ(body.at("mode"), "POSCTL");
  EXPECT_EQ(body.at("landed_state"),
            mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND);
  EXPECT_TRUE(body.at("faults").empty());
}

TEST(MocapRotorWireEncoder, HeartbeatReportsEachOnboardSource) {
  const std::vector<HeartbeatChannel> channels{{"local_pose", 10u, 12, true},
                                                {"extended_state", 2u, -1,
                                                 false}};
  const HeartbeatStats stats{20u, 1u, 4u, 2u};
  std::string payload;
  std::string error;
  ASSERT_TRUE(EncodeHeartbeat("mocap-rotor-01", 8u, 5000, 9000, channels,
                              stats, &payload, &error))
      << error;
  const Json body = Json::parse(payload);
  EXPECT_EQ(body.at("transport"), "zenoh");
  EXPECT_EQ(body.at("channels").size(), 2u);
  EXPECT_EQ(body.at("stats").at("publish_failure"), 1u);
}

} // namespace
} // namespace xgc_mocap_rotor_zenoh_forwarder

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
