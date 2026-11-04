#include "xgc2_ros1_robot_adapter/zenoh_localization_input.hpp"

#include <atomic>
#include <cmath>
#include <mutex>
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

struct VelocityState {
  ZenohLocalizationInput::VelocityCallback callback;
  std::atomic<bool> stopping{false};
};

struct AccelerationState {
  ZenohLocalizationInput::AccelerationCallback callback;
  std::atomic<bool> stopping{false};
};

void decodeTime(const xgc::v1::TimePoint &source, std::int64_t *nanoseconds,
                std::int32_t *clock_domain) {
  *nanoseconds = source.nanoseconds();
  *clock_domain = static_cast<std::int32_t>(source.clock_domain());
}

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
  decodeTime(message.report_time(), &decoded.report_time_nanos,
             &decoded.clock_domain);
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

void onVelocitySample(z_loaned_sample_t *sample, void *context) noexcept {
  if (sample == nullptr || context == nullptr ||
      z_sample_kind(sample) != Z_SAMPLE_KIND_PUT) {
    return;
  }
  auto *state = static_cast<VelocityState *>(context);
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
  xgc2::vrpn_zenoh::v1::Velocity message;
  const bool parsed = message.ParseFromArray(
      z_slice_data(z_loan(slice)), static_cast<int>(z_slice_len(z_loan(slice))));
  z_drop(z_move(slice));
  if (!parsed) {
    return;
  }
  ZenohVelocitySample decoded;
  decodeTime(message.report_time(), &decoded.report_time_nanos,
             &decoded.clock_domain);
  decoded.linear[0] = message.linear_mps().x();
  decoded.linear[1] = message.linear_mps().y();
  decoded.linear[2] = message.linear_mps().z();
  decoded.rotation[0] = message.rotation_over_interval_xyzw().x();
  decoded.rotation[1] = message.rotation_over_interval_xyzw().y();
  decoded.rotation[2] = message.rotation_over_interval_xyzw().z();
  decoded.rotation[3] = message.rotation_over_interval_xyzw().w();
  decoded.interval_seconds = message.interval_seconds();
  try {
    state->callback(decoded);
  } catch (...) {
  }
}

void onAccelerationSample(z_loaned_sample_t *sample, void *context) noexcept {
  if (sample == nullptr || context == nullptr ||
      z_sample_kind(sample) != Z_SAMPLE_KIND_PUT) {
    return;
  }
  auto *state = static_cast<AccelerationState *>(context);
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
  xgc2::vrpn_zenoh::v1::Acceleration message;
  const bool parsed = message.ParseFromArray(
      z_slice_data(z_loan(slice)), static_cast<int>(z_slice_len(z_loan(slice))));
  z_drop(z_move(slice));
  if (!parsed) {
    return;
  }
  ZenohAccelerationSample decoded;
  decodeTime(message.report_time(), &decoded.report_time_nanos,
             &decoded.clock_domain);
  decoded.linear[0] = message.linear_mps2().x();
  decoded.linear[1] = message.linear_mps2().y();
  decoded.linear[2] = message.linear_mps2().z();
  decoded.rotation[0] = message.velocity_rotation_change_xyzw().x();
  decoded.rotation[1] = message.velocity_rotation_change_xyzw().y();
  decoded.rotation[2] = message.velocity_rotation_change_xyzw().z();
  decoded.rotation[3] = message.velocity_rotation_change_xyzw().w();
  decoded.interval_seconds = message.interval_seconds();
  try {
    state->callback(decoded);
  } catch (...) {
  }
}

void onPoseDrop(void *context) { delete static_cast<PoseState *>(context); }
void onVelocityDrop(void *context) {
  delete static_cast<VelocityState *>(context);
}
void onAccelerationDrop(void *context) {
  delete static_cast<AccelerationState *>(context);
}

} // namespace

bool zenohAngularRate(const double rotation_xyzw[4], double interval_seconds,
                      double output[3]) {
  if (rotation_xyzw == nullptr || output == nullptr)
    return false;
  double x = rotation_xyzw[0], y = rotation_xyzw[1], z = rotation_xyzw[2],
         w = rotation_xyzw[3];
  const double norm_squared = x * x + y * y + z * z + w * w;
  if (!std::isfinite(norm_squared) || norm_squared <= 0.0)
    return false;
  const double norm = std::sqrt(norm_squared);
  x /= norm;
  y /= norm;
  z /= norm;
  w /= norm;
  const double sin_half = std::sqrt(x * x + y * y + z * z);
  const double angle = 2.0 * std::atan2(sin_half, w);
  if (!std::isfinite(angle) || !std::isfinite(interval_seconds) ||
      interval_seconds <= 0.0)
    return false;
  if (sin_half < 1.0e-12) {
    output[0] = output[1] = output[2] = 0.0;
    return true;
  }
  const double scale = angle / (sin_half * interval_seconds);
  output[0] = x * scale;
  output[1] = y * scale;
  output[2] = z * scale;
  return std::isfinite(output[0]) && std::isfinite(output[1]) &&
         std::isfinite(output[2]);
}

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
  z_owned_subscriber_t subscribers[3]{};
  bool open[3] = {false, false, false};
  PoseState *pose_state = nullptr;
  VelocityState *velocity_state = nullptr;
  AccelerationState *acceleration_state = nullptr;
  bool session_open = false;
  bool stopped = false;
  std::mutex stop_mutex;

  ~Impl() { Stop(); }

  template <typename State, typename Callback, typename OnSample, typename OnDrop>
  bool Declare(std::size_t index, const char *kind,
                         const ZenohLocalizationBinding &binding,
                         Callback callback, OnSample on_sample, OnDrop on_drop,
                         State **state_out, std::string *error) {
    const std::string key = ZenohLocalizationKey(binding, kind);
    z_view_keyexpr_t expression;
    if (z_view_keyexpr_from_str(&expression, key.c_str()) < 0) {
      if (error != nullptr)
        *error = "invalid Zenoh localization key: " + key;
      return false;
    }
    auto *state = new State{std::move(callback)};
    z_owned_closure_sample_t closure;
    z_closure(&closure, on_sample, on_drop, state);
    if (z_declare_subscriber(z_loan(session), &subscribers[index],
                             z_loan(expression), z_move(closure), nullptr) < 0) {
      if (error != nullptr)
        *error = "cannot subscribe to Zenoh localization key: " + key;
      return false;
    }
    open[index] = true;
    *state_out = state;
    return true;
  }

  void Stop() {
    std::lock_guard<std::mutex> lock(stop_mutex);
    if (stopped)
      return;
    stopped = true;
    if (pose_state != nullptr)
      pose_state->stopping.store(true, std::memory_order_release);
    if (velocity_state != nullptr)
      velocity_state->stopping.store(true, std::memory_order_release);
    if (acceleration_state != nullptr)
      acceleration_state->stopping.store(true, std::memory_order_release);
    for (std::size_t index = 0u; index < 3u; ++index) {
      if (open[index]) {
        // The SDK guarantees closure drop after every in-flight call ends.
        z_drop(z_move(subscribers[index]));
        open[index] = false;
      }
    }
    pose_state = nullptr;
    velocity_state = nullptr;
    acceleration_state = nullptr;
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
                             Callbacks callbacks, std::string *error) {
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
  if (!callbacks.pose && !callbacks.velocity && !callbacks.acceleration) {
    if (error != nullptr)
      *error = "at least one Zenoh localization callback is required";
    return nullptr;
  }

  const std::string config =
      std::string("{\"mode\":\"client\",\"listen\":{\"endpoints\":[]},"
                  "\"connect\":{\"endpoints\":[") +
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

  if (callbacks.pose &&
      !impl->Declare<PoseState>(
          0u, kPoseKind, binding, std::move(callbacks.pose),
          onPoseSample, onPoseDrop, &impl->pose_state, error)) {
    return nullptr;
  }
  if (callbacks.velocity &&
      !impl->Declare<VelocityState>(
          1u, kVelocityKind, binding,
          std::move(callbacks.velocity), onVelocitySample, onVelocityDrop,
          &impl->velocity_state, error)) {
    return nullptr;
  }
  if (callbacks.acceleration &&
      !impl->Declare<AccelerationState>(
          2u, kAccelerationKind, binding,
          std::move(callbacks.acceleration), onAccelerationSample,
          onAccelerationDrop, &impl->acceleration_state, error)) {
    return nullptr;
  }
  return std::unique_ptr<ZenohLocalizationInput>(
      new ZenohLocalizationInput(std::move(impl)));
}

} // namespace xgc2_ros1_robot_adapter
