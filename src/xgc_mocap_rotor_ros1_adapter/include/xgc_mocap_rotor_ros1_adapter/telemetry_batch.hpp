#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

namespace xgc_mocap_rotor_ros1_adapter {

constexpr std::size_t kMaximumTelemetryBatchItems = 16u;
constexpr std::size_t kMaximumTelemetryBatchBytes = 256u * 1024u;

struct TelemetryQueueItem {
  std::uint64_t token = 0u;
  std::string value;
};

struct TelemetryBatch {
  std::vector<std::string> items;
  std::vector<std::uint64_t> tokens;
};

inline TelemetryBatch buildTelemetryBatch(
    const std::deque<TelemetryQueueItem> &queue,
    std::size_t maximum_items = kMaximumTelemetryBatchItems,
    std::size_t maximum_bytes = kMaximumTelemetryBatchBytes) {
  TelemetryBatch batch;
  const std::size_t item_limit =
      std::min(maximum_items, kMaximumTelemetryBatchItems);
  std::size_t bytes = 0u;
  for (const auto &item : queue) {
    if (batch.items.size() >= item_limit ||
        item.value.size() > maximum_bytes - bytes)
      break;
    batch.items.push_back(item.value);
    batch.tokens.push_back(item.token);
    bytes += item.value.size();
  }
  return batch;
}

inline bool telemetryBatchMatchesPrefix(
    const std::deque<TelemetryQueueItem> &queue,
    const std::vector<std::uint64_t> &tokens) {
  if (tokens.size() > queue.size())
    return false;
  for (std::size_t index = 0u; index < tokens.size(); ++index) {
    if (queue[index].token != tokens[index])
      return false;
  }
  return true;
}

} // namespace xgc_mocap_rotor_ros1_adapter
