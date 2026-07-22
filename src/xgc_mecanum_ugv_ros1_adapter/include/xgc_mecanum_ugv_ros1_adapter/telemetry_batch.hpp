#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <vector>

namespace xgc_mecanum_ugv_ros1_adapter {

constexpr std::size_t kMaximumTelemetryBatchItems = 16u;
constexpr std::size_t kMaximumTelemetryBatchBytes = 256u * 1024u;

struct TelemetryQueueItem {
  std::uint64_t token = 0;
  std::string value;
};

struct TelemetryBatch {
  std::vector<std::uint64_t> tokens;
  std::vector<std::string> items;
  std::size_t bytes = 0;
};

inline TelemetryBatch
buildTelemetryBatch(const std::deque<TelemetryQueueItem> &queue,
                    std::size_t maximum_items = kMaximumTelemetryBatchItems,
                    std::size_t maximum_bytes = kMaximumTelemetryBatchBytes) {
  TelemetryBatch batch;
  if (maximum_items == 0u || maximum_bytes == 0u)
    return batch;
  const std::size_t reserve = std::min(queue.size(), maximum_items);
  batch.tokens.reserve(reserve);
  batch.items.reserve(reserve);
  for (const auto &queued : queue) {
    if (batch.items.size() >= maximum_items ||
        queued.value.size() > maximum_bytes - batch.bytes) {
      break;
    }
    batch.tokens.push_back(queued.token);
    batch.items.push_back(queued.value);
    batch.bytes += queued.value.size();
  }
  return batch;
}

inline bool
telemetryBatchMatchesPrefix(const std::deque<TelemetryQueueItem> &queue,
                            const std::vector<std::uint64_t> &tokens) {
  if (tokens.empty() || queue.size() < tokens.size())
    return false;
  for (std::size_t index = 0; index < tokens.size(); ++index) {
    if (queue[index].token != tokens[index])
      return false;
  }
  return true;
}

} // namespace xgc_mecanum_ugv_ros1_adapter
