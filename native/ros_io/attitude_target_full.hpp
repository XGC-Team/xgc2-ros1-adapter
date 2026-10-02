#pragma once

#include "xgc_schemas_v1.h"

// Keep the full mask and ignored payload. Consumers decide which fields to
// use; this transport conversion must not turn a body-rate command into an
// attitude command by dropping IGNORE_ATTITUDE or substituting a quaternion.
template <class Message>
xgc_attitude_target_v2 full_attitude_target(const Message& message, double stamp) {
  xgc_attitude_target_v2 result{};
  result.stamp = stamp;
  result.q_wxyz[0] = message.orientation.w;
  result.q_wxyz[1] = message.orientation.x;
  result.q_wxyz[2] = message.orientation.y;
  result.q_wxyz[3] = message.orientation.z;
  result.body_rate[0] = message.body_rate.x;
  result.body_rate[1] = message.body_rate.y;
  result.body_rate[2] = message.body_rate.z;
  result.thrust = message.thrust;
  result.type_mask = message.type_mask;
  return result;
}

template <class Message>
bool assign_full_attitude_target(const xgc_attitude_target_v2& sample, Message* message) {
  if (sample.type_mask > 255) return false; // never truncate an unknown mask
  message->orientation.w = sample.q_wxyz[0];
  message->orientation.x = sample.q_wxyz[1];
  message->orientation.y = sample.q_wxyz[2];
  message->orientation.z = sample.q_wxyz[3];
  message->body_rate.x = sample.body_rate[0];
  message->body_rate.y = sample.body_rate[1];
  message->body_rate.z = sample.body_rate[2];
  message->thrust = sample.thrust;
  message->type_mask = sample.type_mask;
  return true;
}
