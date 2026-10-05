#include <xgc-ros-runtime-edge/attitude_target_full.hpp>
#include <mavros_msgs/AttitudeTarget.h>
#include <cassert>
#include <cmath>
#include <limits>

int main() {
  mavros_msgs::AttitudeTarget message;
  message.orientation.w = 0.5;
  message.orientation.x = -0.5;
  message.orientation.y = 0.5;
  message.orientation.z = -0.5;
  message.body_rate.x = 0.2;
  message.body_rate.y = -0.4;
  message.body_rate.z = 0.8;
  message.thrust = 0.3f;
  // Every wire mask survives, including combinations unsupported by a
  // particular actuator consumer. The ROS edge must not silently decide.
  for (unsigned mask = 0; mask != 256; ++mask) {
    message.type_mask = mask;
    const auto sample = full_attitude_target(message, 123.5);
    assert(sample.stamp == 123.5 && sample.type_mask == mask);
    assert(sample.q_wxyz[0] == 0.5 && sample.q_wxyz[1] == -0.5);
    assert(sample.q_wxyz[2] == 0.5 && sample.q_wxyz[3] == -0.5);
    assert(sample.body_rate[0] == 0.2 && sample.body_rate[1] == -0.4);
    assert(sample.body_rate[2] == 0.8 && sample.thrust == double(0.3f));
    assert(sample.reserved == 0);
    mavros_msgs::AttitudeTarget roundtrip;
    assert(assign_full_attitude_target(sample, &roundtrip));
    assert(roundtrip.type_mask == mask);
    assert(roundtrip.body_rate.x == message.body_rate.x);
    assert(roundtrip.body_rate.y == message.body_rate.y);
    assert(roundtrip.body_rate.z == message.body_rate.z);
    assert(roundtrip.thrust == message.thrust);
    assert(roundtrip.orientation.w == message.orientation.w);
  }
  message.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
  message.orientation.w = std::numeric_limits<double>::quiet_NaN();
  const auto rates = full_attitude_target(message, 125.0);
  assert(std::isnan(rates.q_wxyz[0]));
  assert(rates.type_mask == mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE);
  assert(rates.body_rate[0] == 0.2);
  auto invalid = rates;
  invalid.type_mask = 256;
  assert(!assign_full_attitude_target(invalid, &message));
}
