#include "xgc2_ros1_robot_adapter/contract.hpp"

namespace xgc2_ros1_robot_adapter {
namespace contract {

const ProfileMetadata *Contract::profiles(std::size_t *count) const {
  if (count != nullptr)
    *count = profile_count_;
  return profiles_;
}

const ProfileMetadata *Contract::profile(const std::string &profile_id) const {
  for (std::size_t index = 0u; index < profile_count_; ++index)
    if (profile_id == profiles_[index].profile_id)
      return &profiles_[index];
  return nullptr;
}

const char *Contract::profileDigest(const std::string &profile_id) const {
  const ProfileMetadata *found = profile(profile_id);
  return found == nullptr ? nullptr : found->digest;
}

const ParameterMetadata *
Contract::profileParameters(const std::string &profile_id,
                            std::size_t *count) const {
  const ProfileMetadata *found = profile(profile_id);
  if (count == nullptr)
    return nullptr;
  *count = found == nullptr ? 0u : found->parameter_count;
  return found == nullptr ? nullptr : found->parameters;
}

bool Contract::parameterMetadata(const std::string &profile_id,
                                 const std::string &name,
                                 ParameterMetadata *out) const {
  std::size_t count = 0u;
  const ParameterMetadata *values = profileParameters(profile_id, &count);
  if (out == nullptr)
    return false;
  for (std::size_t index = 0u; index < count; ++index) {
    if (name == values[index].name) {
      *out = values[index];
      return true;
    }
  }
  return false;
}

const OperationMetadata *
Contract::profileOperations(const std::string &profile_id,
                            std::size_t *count) const {
  const ProfileMetadata *found = profile(profile_id);
  if (count == nullptr)
    return nullptr;
  *count = found == nullptr ? 0u : found->operation_count;
  return found == nullptr ? nullptr : found->operations;
}

bool Contract::operationMetadata(const std::string &profile_id,
                                 const std::string &operation_id,
                                 OperationMetadata *out) const {
  std::size_t count = 0u;
  const OperationMetadata *values = profileOperations(profile_id, &count);
  if (out == nullptr)
    return false;
  for (std::size_t index = 0u; index < count; ++index) {
    if (operation_id == values[index].operation_id) {
      *out = values[index];
      return true;
    }
  }
  return false;
}

const ChannelMetadata *Contract::profileChannels(const std::string &profile_id,
                                                 std::size_t *count) const {
  const ProfileMetadata *found = profile(profile_id);
  if (count == nullptr)
    return nullptr;
  *count = found == nullptr ? 0u : found->channel_count;
  return found == nullptr ? nullptr : found->channels;
}

bool Contract::channelMetadata(const std::string &profile_id,
                               const std::string &channel_id,
                               ChannelMetadata *out) const {
  std::size_t count = 0u;
  const ChannelMetadata *channels = profileChannels(profile_id, &count);
  if (out == nullptr)
    return false;
  for (std::size_t index = 0u; index < count; ++index) {
    if (channel_id == channels[index].channel_id) {
      *out = channels[index];
      return true;
    }
  }
  return false;
}

bool Contract::messageMetadata(std::uint32_t id, MessageMetadata *out) const {
  if (out == nullptr)
    return false;
  for (std::size_t index = 0u; index < message_count_; ++index) {
    if (messages_[index].id == id) {
      *out = messages_[index].metadata;
      return true;
    }
  }
  return false;
}

const EndpointMetadata *channelEndpoint(const ChannelMetadata &channel,
                                        EndpointKind kind,
                                        const std::string &role) {
  for (std::size_t index = 0u; index < channel.endpoint_count; ++index) {
    const EndpointMetadata &endpoint = channel.endpoints[index];
    if (endpoint.kind == kind && role == endpoint.role)
      return &endpoint;
  }
  return nullptr;
}

bool channelObserves(const ChannelMetadata &channel,
                     const std::string &channel_id) {
  for (std::size_t index = 0u; index < channel.observes_count; ++index)
    if (channel_id == channel.observes[index])
      return true;
  return false;
}

const PolicyMetadata *channelPolicy(const ChannelMetadata &channel,
                                    const std::string &key) {
  for (std::size_t index = 0u; index < channel.policy_count; ++index)
    if (key == channel.policy[index].key)
      return &channel.policy[index];
  return nullptr;
}

bool channelPolicyBoolean(const ChannelMetadata &channel, const std::string &key,
                          bool *out) {
  const PolicyMetadata *value = channelPolicy(channel, key);
  if (out == nullptr || value == nullptr ||
      value->kind != PolicyValueKind::kBoolean)
    return false;
  *out = value->boolean_value;
  return true;
}

bool channelPolicyInteger(const ChannelMetadata &channel, const std::string &key,
                          std::int64_t *out) {
  const PolicyMetadata *value = channelPolicy(channel, key);
  if (out == nullptr || value == nullptr ||
      value->kind != PolicyValueKind::kInteger)
    return false;
  *out = value->integer_value;
  return true;
}

bool channelPolicyNumber(const ChannelMetadata &channel, const std::string &key,
                         double *out) {
  const PolicyMetadata *value = channelPolicy(channel, key);
  if (out == nullptr || value == nullptr ||
      value->kind != PolicyValueKind::kNumber)
    return false;
  *out = value->number_value;
  return true;
}

bool channelPolicyString(const ChannelMetadata &channel, const std::string &key,
                         const char **out) {
  const PolicyMetadata *value = channelPolicy(channel, key);
  if (out == nullptr || value == nullptr ||
      value->kind != PolicyValueKind::kString)
    return false;
  *out = value->string_value;
  return true;
}

bool channelPolicyStringArray(const ChannelMetadata &channel,
                              const std::string &key, const char *const **out,
                              std::size_t *count) {
  const PolicyMetadata *value = channelPolicy(channel, key);
  if (out == nullptr || count == nullptr || value == nullptr ||
      value->kind != PolicyValueKind::kStringArray)
    return false;
  *out = value->string_array_value;
  *count = value->string_array_size;
  return true;
}

} // namespace contract
} // namespace xgc2_ros1_robot_adapter
