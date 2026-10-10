#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <type_traits>

namespace xgc_ros1_tools_adapter {

inline constexpr std::uint32_t kServiceHelperProtocolMagic = 0x58325253u;
inline constexpr std::uint16_t kServiceHelperProtocolVersion = 1u;
inline constexpr std::size_t kMaximumServiceNameBytes = 4096u;
inline constexpr std::size_t kServiceMd5Bytes = 32u;
inline constexpr std::size_t kMaximumServicePayloadBytes = 8u * 1024u * 1024u;
inline constexpr std::uint32_t kMaximumServiceWaitMilliseconds = 300000u;

enum class ServiceHelperFrameKind : std::uint16_t {
  kRequest = 1u,
  kReady = 2u,
  kCommit = 3u,
  kResponse = 4u,
};

enum class ServiceHelperReadyStatus : std::uint32_t {
  kReady = 1u,
  kServiceUnavailable = 2u,
  kProtocolError = 3u,
};

enum class ServiceHelperResponseStatus : std::uint32_t {
  kCompleted = 1u,
  kCallFailed = 2u,
  kResponseTooLarge = 3u,
  kProtocolError = 4u,
};

// Frames cross only a private SOCK_STREAM socket between binaries installed by
// this package. Every variable-length section is preceded by this fixed header
// and validated against a hard byte ceiling before allocation or native work.
struct alignas(8) ServiceHelperFrame {
  std::uint32_t magic;
  std::uint16_t version;
  std::uint16_t kind;
  std::uint64_t request_id;
  std::uint32_t status;
  std::uint32_t service_name_bytes;
  std::uint32_t service_md5_bytes;
  std::uint32_t payload_bytes;
  std::uint32_t wait_for_service_ms;
  std::array<std::uint8_t, 28u> reserved;
};

static_assert(sizeof(ServiceHelperFrame) == 64u,
              "service helper frame layout changed");
static_assert(std::is_trivially_copyable<ServiceHelperFrame>::value,
              "service helper frame must be trivially copyable");

inline bool serviceHelperBytesAreZero(const std::uint8_t* begin,
                                      std::size_t size) {
  for (std::size_t index = 0; index < size; ++index) {
    if (begin[index] != 0u) {
      return false;
    }
  }
  return true;
}

inline bool validServiceMd5(const std::string& value) {
  if (value.size() != kServiceMd5Bytes) {
    return false;
  }
  for (const unsigned char character : value) {
    const bool decimal = character >= '0' && character <= '9';
    const bool lower_hex = character >= 'a' && character <= 'f';
    const bool upper_hex = character >= 'A' && character <= 'F';
    if (!decimal && !lower_hex && !upper_hex) {
      return false;
    }
  }
  return true;
}

inline ServiceHelperFrame makeServiceHelperRequest(
    std::uint64_t request_id, std::size_t service_name_bytes,
    std::size_t service_md5_bytes, std::size_t payload_bytes,
    std::uint32_t wait_for_service_ms) {
  ServiceHelperFrame frame{};
  frame.magic = kServiceHelperProtocolMagic;
  frame.version = kServiceHelperProtocolVersion;
  frame.kind = static_cast<std::uint16_t>(ServiceHelperFrameKind::kRequest);
  frame.request_id = request_id;
  frame.service_name_bytes = static_cast<std::uint32_t>(service_name_bytes);
  frame.service_md5_bytes = static_cast<std::uint32_t>(service_md5_bytes);
  frame.payload_bytes = static_cast<std::uint32_t>(payload_bytes);
  frame.wait_for_service_ms = wait_for_service_ms;
  return frame;
}

inline ServiceHelperFrame makeServiceHelperReady(
    std::uint64_t request_id, ServiceHelperReadyStatus status) {
  ServiceHelperFrame frame{};
  frame.magic = kServiceHelperProtocolMagic;
  frame.version = kServiceHelperProtocolVersion;
  frame.kind = static_cast<std::uint16_t>(ServiceHelperFrameKind::kReady);
  frame.request_id = request_id;
  frame.status = static_cast<std::uint32_t>(status);
  return frame;
}

inline ServiceHelperFrame makeServiceHelperCommit(std::uint64_t request_id) {
  ServiceHelperFrame frame{};
  frame.magic = kServiceHelperProtocolMagic;
  frame.version = kServiceHelperProtocolVersion;
  frame.kind = static_cast<std::uint16_t>(ServiceHelperFrameKind::kCommit);
  frame.request_id = request_id;
  return frame;
}

inline ServiceHelperFrame makeServiceHelperResponse(
    std::uint64_t request_id, ServiceHelperResponseStatus status,
    std::size_t payload_bytes = 0u) {
  ServiceHelperFrame frame{};
  frame.magic = kServiceHelperProtocolMagic;
  frame.version = kServiceHelperProtocolVersion;
  frame.kind = static_cast<std::uint16_t>(ServiceHelperFrameKind::kResponse);
  frame.request_id = request_id;
  frame.status = static_cast<std::uint32_t>(status);
  frame.payload_bytes = static_cast<std::uint32_t>(payload_bytes);
  return frame;
}

inline bool validServiceHelperFrameBase(const ServiceHelperFrame& frame) {
  return frame.magic == kServiceHelperProtocolMagic &&
         frame.version == kServiceHelperProtocolVersion &&
         frame.request_id != 0u &&
         serviceHelperBytesAreZero(frame.reserved.data(),
                                   frame.reserved.size());
}

inline bool validateServiceHelperRequest(const ServiceHelperFrame& frame) {
  return validServiceHelperFrameBase(frame) &&
         frame.kind ==
             static_cast<std::uint16_t>(ServiceHelperFrameKind::kRequest) &&
         frame.status == 0u && frame.service_name_bytes > 0u &&
         frame.service_name_bytes <= kMaximumServiceNameBytes &&
         frame.service_md5_bytes == kServiceMd5Bytes &&
         frame.payload_bytes <= kMaximumServicePayloadBytes &&
         frame.wait_for_service_ms <= kMaximumServiceWaitMilliseconds;
}

inline bool validateServiceHelperReady(const ServiceHelperFrame& frame,
                                       std::uint64_t request_id) {
  if (!validServiceHelperFrameBase(frame) || frame.request_id != request_id ||
      frame.kind !=
          static_cast<std::uint16_t>(ServiceHelperFrameKind::kReady) ||
      frame.service_name_bytes != 0u || frame.service_md5_bytes != 0u ||
      frame.payload_bytes != 0u || frame.wait_for_service_ms != 0u) {
    return false;
  }
  const auto status = static_cast<ServiceHelperReadyStatus>(frame.status);
  return status == ServiceHelperReadyStatus::kReady ||
         status == ServiceHelperReadyStatus::kServiceUnavailable ||
         status == ServiceHelperReadyStatus::kProtocolError;
}

inline bool validateServiceHelperCommit(const ServiceHelperFrame& frame,
                                        std::uint64_t request_id) {
  return validServiceHelperFrameBase(frame) && frame.request_id == request_id &&
         frame.kind ==
             static_cast<std::uint16_t>(ServiceHelperFrameKind::kCommit) &&
         frame.status == 0u && frame.service_name_bytes == 0u &&
         frame.service_md5_bytes == 0u && frame.payload_bytes == 0u &&
         frame.wait_for_service_ms == 0u;
}

inline bool validateServiceHelperResponse(const ServiceHelperFrame& frame,
                                          std::uint64_t request_id) {
  if (!validServiceHelperFrameBase(frame) || frame.request_id != request_id ||
      frame.kind !=
          static_cast<std::uint16_t>(ServiceHelperFrameKind::kResponse) ||
      frame.service_name_bytes != 0u || frame.service_md5_bytes != 0u ||
      frame.wait_for_service_ms != 0u ||
      frame.payload_bytes > kMaximumServicePayloadBytes) {
    return false;
  }
  const auto status = static_cast<ServiceHelperResponseStatus>(frame.status);
  if (status == ServiceHelperResponseStatus::kCompleted) {
    return true;
  }
  return frame.payload_bytes == 0u &&
         (status == ServiceHelperResponseStatus::kCallFailed ||
          status == ServiceHelperResponseStatus::kResponseTooLarge ||
          status == ServiceHelperResponseStatus::kProtocolError);
}

}  // namespace xgc_ros1_tools_adapter
