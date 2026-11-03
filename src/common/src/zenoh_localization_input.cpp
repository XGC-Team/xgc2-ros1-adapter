#include "xgc2_ros1_robot_adapter/zenoh_localization_input.hpp"

#include <atomic>
#include <mutex>
#include <cstdio>
#include <stdexcept>
#include <utility>

#include <zenoh.h>

#include "xgc2/vrpn_zenoh/v1/tracker.pb.h"

namespace xgc2_ros1_robot_adapter {
namespace {

constexpr char kKeyPrefix[] = "xgc2/localization/vrpn/v1";
constexpr char kPoseKind[] = "pose";
constexpr char kVelocityKind[] = "velocity";
constexpr char kAccelerationKind[] = "acceleration";

std::string hexLower(const std::string &raw) {
  static const char digits[] = "0123456789abcdef";
  std::string output;
  output.reserve(raw.size() * 2u);
  for (const unsigned char byte : raw) {
    output.push_back(digits[byte >> 4u]);
    output.push_back(digits[byte & 0x0fu]);
  }
  return output;
}

std::string jsonString(const std::string &value) {
  std::string output;
  output.reserve(value.size() + 2u);
  output.push_back('"');
  for (const char character : value) {
    switch (character) {
    case '"':
      output += "\\\"";
      break;
    case '\\':
      output += "\\\\";
      break;
    case '\n':
      output += "\\n";
      break;
    case '\r':
      output += "\\r";
      break;
    case '\t':
      output += "\\t";
      break;
    default:
      output.push_back(character);
      break;
    }
  }
  output.push_back('"');
  return output;
}

struct PoseState {
  ZenohLocalizationInput::PoseCallback callback;
  std::atomic<bool> stopping{false};
};

void onPoseSample(z_loaned_sample_t *sample, void *context) noexcept {
  if (sample == nullptr || context == nullptr ||
      z_sample_kind(sample) != Z_SAMPLE_KIND_PUT) {
    return;
  }
  auto *state = static_cast<PoseState *>(context);
  if (state->stopping.load(std::memory_order_acquire)) {
    return;
  }
  const auto *bytes = z_sample_payload(sample);
  if (bytes == nullptr || z_bytes_len(bytes) == 0u) {
    return;
  }
  z_owned_slice_t slice;
  if (z_bytes_to_slice(bytes, &slice) < 0) {
    return;
  }
  xgc2::vrpn_zenoh::v1::Pose message;
  const bool parsed = message.ParseFromArray(
      z_slice_data(z_loan(slice)), static_cast<int>(z_slice_len(z_loan(slice))));
  z_drop(z_move(slice));
  if (!parsed) {
    return;
  }
  ZenohPoseSample decoded;
  decoded.report_time_nanos = message.report_time().nanoseconds();
  decoded.clock_domain = static_cast<std::int32_t>(
      message.report_time().clock_domain());
  decoded.position[0] = message.position_m().x();
  decoded.position[1] = message.position_m().y();
  decoded.position[2] = message.position_m().z();
  decoded.orientation[0] = message.orientation_xyzw().x();
  decoded.orientation[1] = message.orientation_xyzw().y();
  decoded.orientation[2] = message.orientation_xyzw().z();
  decoded.orientation[3] = message.orientation_xyzw().w();
  try {
    state->callback(decoded);
  } catch (...) {
  }
}

void onPoseDrop(void *context) { delete static_cast<PoseState *>(context); }

} // namespace

std::string ZenohLocalizationKey(const ZenohLocalizationBinding &binding,
                                 const char *kind) {
  if (kind == nullptr || kind[0] == '\0') {
    throw std::invalid_argument("Zenoh localization kind is required");
  }
  return std::string(kKeyPrefix) + "/" + hexLower(binding.source) + "/" +
         hexLower(binding.tracker) + "/" + std::to_string(binding.sensor) +
         "/" + kind;
}

struct ZenohLocalizationInput::Impl {
  z_owned_session_t session{};
  z_owned_subscriber_t subscriber{};
  PoseState *state = nullptr;
  bool session_open = false;
  bool subscriber_open = false;
  bool stopped = false;
  std::mutex stop_mutex;

  ~Impl() { Stop(); }

  void Stop() {
    std::lock_guard<std::mutex> lock(stop_mutex);
    if (stopped)
      return;
    stopped = true;
    if (state != nullptr)
      state->stopping.store(true, std::memory_order_release);
    if (subscriber_open) {
      // Dropping the subscriber drops its closure. The SDK guarantees drop
      // runs only after every in-flight call has ended, so the callback state
      // cannot be deleted while a callback still uses it.
      z_drop(z_move(subscriber));
      subscriber_open = false;
    }
    // onPoseDrop owns and deletes the callback state; never touch `state`
    // after the subscriber drop, including from the destructor's second Stop.
    state = nullptr;
    if (session_open) {
      z_drop(z_move(session));
      session_open = false;
    }
  }
};

ZenohLocalizationInput::ZenohLocalizationInput(std::unique_ptr<Impl> impl)
    : impl_(std::move(impl)) {}

ZenohLocalizationInput::~ZenohLocalizationInput() = default;

void ZenohLocalizationInput::Stop() {
  if (impl_ != nullptr) {
    impl_->Stop();
  }
}

std::unique_ptr<ZenohLocalizationInput>
ZenohLocalizationInput::Open(const ZenohLocalizationBinding &binding,
                             PoseCallback callback, std::string *error) {
  if (binding.endpoint.empty()) {
    if (error != nullptr)
      *error = "Zenoh localization endpoint is required";
    return nullptr;
  }
  if (binding.source.empty() || binding.tracker.empty()) {
    if (error != nullptr)
      *error = "Zenoh localization source and tracker are required";
    return nullptr;
  }
  if (!callback) {
    if (error != nullptr)
      *error = "Zenoh localization pose callback is required";
    return nullptr;
  }

  std::string key;
  try {
    key = ZenohLocalizationKey(binding, kPoseKind);
  } catch (const std::exception &exception) {
    if (error != nullptr)
      *error = exception.what();
    return nullptr;
  }

  const std::string config =
      std::string("{\"mode\":\"peer\",\"connect\":{\"endpoints\":[") +
      jsonString(binding.endpoint) +
      "]},\"scouting\":{\"multicast\":{\"enabled\":false},"
      "\"gossip\":{\"enabled\":false}}}";
  z_owned_config_t zenoh_config;
  if (zc_config_from_str(&zenoh_config, config.c_str()) < 0) {
    if (error != nullptr)
      *error = "invalid Zenoh localization configuration";
    return nullptr;
  }

  auto impl = std::unique_ptr<Impl>(new Impl());
  if (z_open(&impl->session, z_move(zenoh_config), nullptr) < 0) {
    if (error != nullptr)
      *error = "cannot open Zenoh localization session";
    return nullptr;
  }
  impl->session_open = true;

  z_view_keyexpr_t expression;
  if (z_view_keyexpr_from_str(&expression, key.c_str()) < 0) {
    if (error != nullptr)
      *error = "invalid Zenoh localization key: " + key;
    return nullptr;
  }

  auto *state = new PoseState{std::move(callback)};
  z_owned_closure_sample_t closure;
  z_closure(&closure, onPoseSample, onPoseDrop, state);
  if (z_declare_subscriber(z_loan(impl->session), &impl->subscriber,
                           z_loan(expression), z_move(closure), nullptr) < 0) {
    if (error != nullptr)
      *error = "cannot subscribe to Zenoh localization key: " + key;
    return nullptr;
  }
  impl->subscriber_open = true;
  impl->state = state;
  return std::unique_ptr<ZenohLocalizationInput>(
      new ZenohLocalizationInput(std::move(impl)));
}

} // namespace xgc2_ros1_robot_adapter
