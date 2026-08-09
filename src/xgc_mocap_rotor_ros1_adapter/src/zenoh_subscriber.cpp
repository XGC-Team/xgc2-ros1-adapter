#include "xgc_mocap_rotor_ros1_adapter/zenoh_subscriber.hpp"

#include <utility>

#include "xgc_mocap_rotor_ros1_adapter/wire_contract.hpp"

namespace xgc_mocap_rotor_ros1_adapter {
namespace {

bool fail(std::string *error, const std::string &message) {
  if (error != nullptr)
    *error = message;
  return false;
}

} // namespace

ZenohSubscriber::ZenohSubscriber(Handler handler) : handler_(std::move(handler)) {
  z_internal_null(&session_);
  z_internal_null(&subscriber_);
}

ZenohSubscriber::~ZenohSubscriber() { Stop(); }

bool ZenohSubscriber::Start(const std::string &listen_endpoint,
                            std::string *error) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (running_)
    return true;
  if (!handler_)
    return fail(error, "Zenoh sample handler is unavailable");
  if (!ValidateZenohListenEndpoint(listen_endpoint, error))
    return false;

  zc_init_log_from_env_or("error");
  z_owned_config_t config;
  z_internal_null(&config);
  if (z_config_default(&config) < 0)
    return fail(error, "cannot create the default Zenoh configuration");

  const std::string endpoints = "['" + listen_endpoint + "']";
  if (zc_config_insert_json5(z_loan_mut(config), Z_CONFIG_MODE_KEY,
                             "'peer'") < 0 ||
      zc_config_insert_json5(z_loan_mut(config), Z_CONFIG_LISTEN_KEY,
                             endpoints.c_str()) < 0 ||
      zc_config_insert_json5(z_loan_mut(config),
                             Z_CONFIG_MULTICAST_SCOUTING_KEY, "false") < 0 ||
      zc_config_insert_json5(z_loan_mut(config), "scouting/gossip/enabled",
                             "false") < 0) {
    z_drop(z_move(config));
    return fail(error, "cannot build the bounded Zenoh peer configuration");
  }
  if (z_open(&session_, z_move(config), nullptr) < 0)
    return fail(error, "cannot open the Mocap Rotor Zenoh peer session");

  z_view_keyexpr_t key_expression;
  if (z_view_keyexpr_from_str(&key_expression, "xgc2/*/up/**") < 0) {
    z_drop(z_move(session_));
    return fail(error, "cannot construct the Mocap Rotor Zenoh key selector");
  }
  z_owned_closure_sample_t callback;
  z_closure(&callback, HandleSample, nullptr, this);
  if (z_declare_subscriber(z_loan(session_), &subscriber_,
                           z_loan(key_expression), z_move(callback),
                           nullptr) < 0) {
    z_drop(z_move(session_));
    return fail(error, "cannot declare the Mocap Rotor Zenoh subscriber");
  }
  running_ = true;
  return true;
}

void ZenohSubscriber::Stop() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (z_internal_check(subscriber_))
    z_drop(z_move(subscriber_));
  if (z_internal_check(session_))
    z_drop(z_move(session_));
  running_ = false;
}

bool ZenohSubscriber::running() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return running_;
}

void ZenohSubscriber::HandleSample(z_loaned_sample_t *sample, void *context) {
  if (context != nullptr)
    static_cast<ZenohSubscriber *>(context)->onSample(sample);
}

void ZenohSubscriber::onSample(z_loaned_sample_t *sample) noexcept {
  try {
    if (sample == nullptr || z_sample_kind(sample) != Z_SAMPLE_KIND_PUT)
      return;
    z_view_string_t key;
    z_keyexpr_as_view_string(z_sample_keyexpr(sample), &key);
    const auto *key_string = z_loan(key);
    if (key_string == nullptr)
      return;

    z_owned_string_t payload;
    z_internal_null(&payload);
    if (z_bytes_to_string(z_sample_payload(sample), &payload) < 0)
      return;
    const auto *payload_string = z_loan(payload);
    if (payload_string != nullptr) {
      handler_(std::string(z_string_data(key_string), z_string_len(key_string)),
               std::string(z_string_data(payload_string),
                           z_string_len(payload_string)));
    }
    z_drop(z_move(payload));
  } catch (...) {
    // Foreign callbacks must never unwind through the Zenoh C ABI. Runtime
    // decoding records and reports malformed samples at the dispatch layer.
  }
}

} // namespace xgc_mocap_rotor_ros1_adapter
