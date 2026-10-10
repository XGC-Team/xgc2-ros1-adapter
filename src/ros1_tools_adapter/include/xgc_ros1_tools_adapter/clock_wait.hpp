#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>

namespace xgc_ros1_tools_adapter {

// Decimal seconds are rounded up to the next representable ROS nanosecond.
// No binary floating-point residue enters the threshold comparison.
inline std::uint64_t clockTarget(const std::string& text) {
  const auto point = text.find('.');
  const auto whole = text.substr(0, point);
  if (whole.empty() || text.size() > 128) {
    throw std::invalid_argument(
        "target must be bounded nonnegative decimal seconds");
  }
  std::uint64_t seconds = 0;
  for (const char digit : whole) {
    if (digit < '0' || digit > '9' || seconds > UINT32_MAX / 10ULL) {
      throw std::invalid_argument("target is outside ROS time");
    }
    seconds = seconds * 10 + static_cast<unsigned>(digit - '0');
    if (seconds > UINT32_MAX)
      throw std::invalid_argument("target is outside ROS time");
  }
  std::uint64_t nanos = 0;
  if (point != std::string::npos) {
    const auto fraction = text.substr(point + 1);
    if (fraction.empty())
      throw std::invalid_argument("target has an empty fraction");
    bool round_up = false;
    for (std::size_t i = 0; i < fraction.size(); ++i) {
      const char digit = fraction[i];
      if (digit < '0' || digit > '9')
        throw std::invalid_argument("invalid decimal target");
      if (i < 9) {
        nanos = nanos * 10 + static_cast<unsigned>(digit - '0');
      } else {
        round_up = round_up || digit != '0';
      }
    }
    for (std::size_t i = fraction.size(); i < 9; ++i) nanos *= 10;
    nanos += round_up ? 1 : 0;
  }
  if (seconds == UINT32_MAX && nanos != 0)
    throw std::invalid_argument("target is outside ROS time");
  return seconds * 1000000000ULL + nanos;
}

class ClockWait {
 public:
  explicit ClockWait(const std::string& seconds)
      : target_(clockTarget(seconds)) {}

  bool observe(std::uint32_t sec, std::uint32_t nsec) {
    if (nsec >= 1000000000U)
      throw std::runtime_error("invalid clock nanoseconds");
    const auto stamp = static_cast<std::uint64_t>(sec) * 1000000000ULL + nsec;
    if (observed_ && stamp < previous_)
      throw std::runtime_error("simulation clock moved backwards");
    previous_ = stamp;
    observed_ = true;
    return stamp >= target_;
  }

  bool observed() const { return observed_; }

 private:
  const std::uint64_t target_;
  std::uint64_t previous_ = 0;
  bool observed_ = false;
};

}  // namespace xgc_ros1_tools_adapter
