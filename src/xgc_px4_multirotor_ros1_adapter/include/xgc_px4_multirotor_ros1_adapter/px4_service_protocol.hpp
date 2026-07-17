#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <type_traits>

namespace xgc_px4_multirotor_ros1_adapter {

constexpr std::uint32_t kPx4ServiceProtocolMagic = 0x58325034u;
constexpr std::uint16_t kPx4ServiceProtocolVersion = 1u;
constexpr std::size_t kPx4ServiceModeCapacity = 31u;

enum class Px4ServiceOperation : std::uint16_t {
  kSetArmed = 1u,
  kSetMode = 2u,
  kRebootAutopilot = 3u,
};

enum class Px4ServiceResponseStatus : std::uint16_t {
  kCompleted = 1u,
  kCallFailed = 2u,
  kProtocolError = 3u,
};

constexpr std::uint16_t kPx4ServiceResponseHasNativeResult = 1u << 0u;

// These frames cross only a local SOCK_SEQPACKET socket between two binaries
// built from this package. Explicit fields and sizes make version skew and
// truncated packets detectable before any result is interpreted.
struct alignas(8) Px4ServiceRequestFrame {
  std::uint32_t magic;
  std::uint16_t version;
  std::uint16_t operation;
  std::uint64_t request_id;
  std::uint8_t armed;
  std::array<char, kPx4ServiceModeCapacity> mode;
  std::array<std::uint8_t, 16u> reserved;
};

struct alignas(8) Px4ServiceResponseFrame {
  std::uint32_t magic;
  std::uint16_t version;
  std::uint16_t status;
  std::uint64_t request_id;
  std::uint16_t operation;
  std::uint16_t flags;
  std::uint8_t logical_success;
  std::uint8_t native_result;
  std::array<std::uint8_t, 10u> reserved;
};

static_assert(sizeof(Px4ServiceRequestFrame) == 64u,
              "PX4 helper request frame layout changed");
static_assert(sizeof(Px4ServiceResponseFrame) == 32u,
              "PX4 helper response frame layout changed");
static_assert(std::is_trivially_copyable<Px4ServiceRequestFrame>::value,
              "PX4 helper request frame must be trivially copyable");
static_assert(std::is_trivially_copyable<Px4ServiceResponseFrame>::value,
              "PX4 helper response frame must be trivially copyable");

inline bool px4ServiceBytesAreZero(const std::uint8_t *begin,
                                   std::size_t size) {
  for (std::size_t index = 0; index < size; ++index) {
    if (begin[index] != 0u)
      return false;
  }
  return true;
}

inline bool px4ServiceCharsAreZero(const char *begin, std::size_t size) {
  for (std::size_t index = 0; index < size; ++index) {
    if (begin[index] != '\0')
      return false;
  }
  return true;
}

inline bool px4ServiceStringIsCanonical(const char *begin, std::size_t size) {
  bool terminated = false;
  for (std::size_t index = 0; index < size; ++index) {
    if (begin[index] == '\0') {
      terminated = true;
    } else if (terminated) {
      return false;
    }
  }
  return terminated;
}

inline Px4ServiceRequestFrame makePx4SetArmedRequest(std::uint64_t request_id,
                                                     bool armed) {
  Px4ServiceRequestFrame frame{};
  frame.magic = kPx4ServiceProtocolMagic;
  frame.version = kPx4ServiceProtocolVersion;
  frame.operation = static_cast<std::uint16_t>(Px4ServiceOperation::kSetArmed);
  frame.request_id = request_id;
  frame.armed = armed ? 1u : 0u;
  return frame;
}

inline bool makePx4SetModeRequest(std::uint64_t request_id,
                                  const std::string &mode,
                                  Px4ServiceRequestFrame *frame) {
  if (frame == nullptr || mode.empty() ||
      mode.size() >= kPx4ServiceModeCapacity) {
    return false;
  }
  *frame = Px4ServiceRequestFrame{};
  frame->magic = kPx4ServiceProtocolMagic;
  frame->version = kPx4ServiceProtocolVersion;
  frame->operation = static_cast<std::uint16_t>(Px4ServiceOperation::kSetMode);
  frame->request_id = request_id;
  std::memcpy(frame->mode.data(), mode.data(), mode.size());
  return true;
}

inline Px4ServiceRequestFrame makePx4RebootRequest(std::uint64_t request_id) {
  Px4ServiceRequestFrame frame{};
  frame.magic = kPx4ServiceProtocolMagic;
  frame.version = kPx4ServiceProtocolVersion;
  frame.operation =
      static_cast<std::uint16_t>(Px4ServiceOperation::kRebootAutopilot);
  frame.request_id = request_id;
  return frame;
}

inline bool validatePx4ServiceRequest(const Px4ServiceRequestFrame &frame) {
  if (frame.magic != kPx4ServiceProtocolMagic ||
      frame.version != kPx4ServiceProtocolVersion || frame.request_id == 0u ||
      !px4ServiceBytesAreZero(frame.reserved.data(), frame.reserved.size())) {
    return false;
  }

  const auto operation = static_cast<Px4ServiceOperation>(frame.operation);
  switch (operation) {
  case Px4ServiceOperation::kSetArmed:
    return frame.armed <= 1u &&
           px4ServiceCharsAreZero(frame.mode.data(), frame.mode.size());
  case Px4ServiceOperation::kSetMode: {
    if (frame.armed != 0u || frame.mode.front() == '\0' ||
        frame.mode.back() != '\0') {
      return false;
    }
    return px4ServiceStringIsCanonical(frame.mode.data(), frame.mode.size());
  }
  case Px4ServiceOperation::kRebootAutopilot:
    return frame.armed == 0u &&
           px4ServiceCharsAreZero(frame.mode.data(), frame.mode.size());
  }
  return false;
}

inline std::string px4ServiceRequestMode(const Px4ServiceRequestFrame &frame) {
  const void *terminator =
      std::memchr(frame.mode.data(), '\0', frame.mode.size());
  if (terminator == nullptr)
    return std::string();
  const auto *end = static_cast<const char *>(terminator);
  return std::string(frame.mode.data(), end);
}

inline Px4ServiceResponseFrame makePx4ServiceResponse(
    const Px4ServiceRequestFrame &request, Px4ServiceResponseStatus status,
    bool logical_success = false, bool has_native_result = false,
    std::uint8_t native_result = 0u) {
  Px4ServiceResponseFrame frame{};
  frame.magic = kPx4ServiceProtocolMagic;
  frame.version = kPx4ServiceProtocolVersion;
  frame.status = static_cast<std::uint16_t>(status);
  frame.request_id = request.request_id;
  frame.operation = request.operation;
  frame.flags = has_native_result ? kPx4ServiceResponseHasNativeResult : 0u;
  frame.logical_success = logical_success ? 1u : 0u;
  frame.native_result = native_result;
  return frame;
}

inline bool validatePx4ServiceResponse(const Px4ServiceResponseFrame &frame,
                                       const Px4ServiceRequestFrame &request) {
  if (frame.magic != kPx4ServiceProtocolMagic ||
      frame.version != kPx4ServiceProtocolVersion ||
      frame.request_id != request.request_id ||
      frame.operation != request.operation || frame.logical_success > 1u ||
      (frame.flags & ~kPx4ServiceResponseHasNativeResult) != 0u ||
      !px4ServiceBytesAreZero(frame.reserved.data(), frame.reserved.size())) {
    return false;
  }
  const auto status = static_cast<Px4ServiceResponseStatus>(frame.status);
  if (status != Px4ServiceResponseStatus::kCompleted &&
      status != Px4ServiceResponseStatus::kCallFailed &&
      status != Px4ServiceResponseStatus::kProtocolError) {
    return false;
  }
  if (status != Px4ServiceResponseStatus::kCompleted) {
    return frame.flags == 0u && frame.logical_success == 0u &&
           frame.native_result == 0u;
  }
  const auto operation = static_cast<Px4ServiceOperation>(frame.operation);
  const bool has_native_result =
      (frame.flags & kPx4ServiceResponseHasNativeResult) != 0u;
  if (operation == Px4ServiceOperation::kSetMode)
    return !has_native_result && frame.native_result == 0u;
  return (operation == Px4ServiceOperation::kSetArmed ||
          operation == Px4ServiceOperation::kRebootAutopilot) &&
         has_native_result;
}

} // namespace xgc_px4_multirotor_ros1_adapter
