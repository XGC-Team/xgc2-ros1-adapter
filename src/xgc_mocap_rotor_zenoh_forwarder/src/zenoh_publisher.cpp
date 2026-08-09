#include "xgc_mocap_rotor_zenoh_forwarder/zenoh_publisher.hpp"

namespace xgc_mocap_rotor_zenoh_forwarder {
namespace {

bool fail(std::string *error, const std::string &message) {
  if (error != nullptr)
    *error = message;
  return false;
}

} // namespace

ZenohPublisher::ZenohPublisher() { z_internal_null(&session_); }

ZenohPublisher::~ZenohPublisher() { Stop(); }

bool ZenohPublisher::Start(const std::string &robot_id,
                           const std::string &connect_endpoint,
                           std::string *error) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (running_)
    return true;
  if (!ValidateRobotId(robot_id, error) ||
      !ValidateZenohConnectEndpoint(connect_endpoint, error)) {
    return false;
  }

  zc_init_log_from_env_or("error");
  z_owned_config_t config;
  z_internal_null(&config);
  if (z_config_default(&config) < 0)
    return fail(error, "cannot create the default Zenoh configuration");
  const std::string endpoints = "['" + connect_endpoint + "']";
  if (zc_config_insert_json5(z_loan_mut(config), Z_CONFIG_MODE_KEY,
                             "'client'") < 0 ||
      zc_config_insert_json5(z_loan_mut(config), Z_CONFIG_CONNECT_KEY,
                             endpoints.c_str()) < 0 ||
      zc_config_insert_json5(z_loan_mut(config),
                             Z_CONFIG_MULTICAST_SCOUTING_KEY, "false") < 0 ||
      zc_config_insert_json5(z_loan_mut(config), "scouting/gossip/enabled",
                             "false") < 0) {
    z_drop(z_move(config));
    return fail(error, "cannot build the bounded Zenoh client configuration");
  }
  if (z_open(&session_, z_move(config), nullptr) < 0)
    return fail(error, "cannot open the Mocap Rotor Zenoh client session");
  robot_id_ = robot_id;
  running_ = true;
  return true;
}

bool ZenohPublisher::Put(UplinkChannel channel, const std::string &payload,
                         std::string *error) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!running_ || !z_internal_check(session_))
    return fail(error, "Mocap Rotor Zenoh client is not running");
  if (payload.empty() || payload.size() > MaximumPayloadBytes(channel))
    return fail(error, "wire payload is empty or exceeds its channel bound");
  const std::string key = UplinkKey(robot_id_, channel);
  z_view_keyexpr_t key_expression;
  if (z_view_keyexpr_from_str(&key_expression, key.c_str()) < 0)
    return fail(error, "cannot construct the fixed Mocap Rotor uplink key");
  z_owned_bytes_t bytes;
  z_bytes_copy_from_buf(&bytes,
                        reinterpret_cast<const std::uint8_t *>(payload.data()),
                        payload.size());
  if (z_put(z_loan(session_), z_loan(key_expression), z_move(bytes), nullptr) <
      0) {
    return fail(error, "Zenoh put failed for " + key);
  }
  return true;
}

void ZenohPublisher::Stop() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (z_internal_check(session_))
    z_drop(z_move(session_));
  robot_id_.clear();
  running_ = false;
}

bool ZenohPublisher::running() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return running_;
}

} // namespace xgc_mocap_rotor_zenoh_forwarder
