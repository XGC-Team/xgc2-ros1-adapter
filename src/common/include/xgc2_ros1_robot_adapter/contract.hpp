#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

// Metadata of the installed robot profiles. tools/generate_contract_metadata.py
// validates the repository-owned profile YAML against the protobuf registry and
// emits the tables below; the lookups are ordinary code shared by all robot
// types, so one library serves every provider.
namespace xgc2_ros1_robot_adapter {
namespace contract {

enum class ParameterType {
  kString,
  kInteger,
  kNumber,
  kBoolean,
};

enum class ChannelKind {
  kStreamOut,
  kOperation,
};

enum class EndpointKind {
  kInput,
  kOutput,
  kService,
};

enum class EndpointScope {
  kRobotNamespace,
  kGlobal,
};

enum class PolicyValueKind {
  kString,
  kInteger,
  kNumber,
  kBoolean,
  kStringArray,
};

struct MessageMetadata {
  const char *type_name;
  std::uint32_t version;
  std::uint64_t fingerprint;
};

struct MessageEntry {
  std::uint32_t id;
  MessageMetadata metadata;
};

struct ParameterMetadata {
  const char *name;
  ParameterType type;
  bool required;
  const char *pattern;
  const char *description;
};

struct EndpointMetadata {
  EndpointKind kind;
  const char *role;
  const char *name_template;
  const char *ros_type;
  EndpointScope scope;
};

struct PolicyMetadata {
  const char *key;
  PolicyValueKind kind;
  const char *string_value;
  std::int64_t integer_value;
  double number_value;
  bool boolean_value;
  const char *const *string_array_value;
  std::size_t string_array_size;
};

struct OperationContractMetadata {
  const char *side_effect;
  const char *idempotency;
  bool cancellation_supported;
  bool deadline_required;
  // "release", "hold" or "stop" when the operation makes the robot safer and
  // so stays dispatchable after an unknown result; "none" for every other
  // operation, "" for a stream.
  const char *safety;
};

struct OperationMetadata {
  const char *operation_id;
  const char *channel_id;
  std::uint32_t timeout_millis;
  const char *parameter_schema_json;
};

struct ChannelMetadata {
  const char *channel_id;
  ChannelKind kind;
  const char *processor;
  const char *operation_id;
  std::uint32_t input_message_id;
  std::uint32_t output_message_id;
  double output_rate_hz;
  std::uint32_t operation_timeout_millis;
  std::uint32_t stale_after_millis;
  const EndpointMetadata *endpoints;
  std::size_t endpoint_count;
  const char *const *observes;
  std::size_t observes_count;
  const PolicyMetadata *policy;
  std::size_t policy_count;
  OperationContractMetadata operation_contract;
};

struct ProfileMetadata {
  const char *profile_id;
  const char *digest;
  const ParameterMetadata *parameters;
  std::size_t parameter_count;
  const OperationMetadata *operations;
  std::size_t operation_count;
  const ChannelMetadata *channels;
  std::size_t channel_count;
};

// The generated contract of one provider definition (one robot type).
class Contract {
public:
  Contract(const char *provider_definition_id,
           std::uint64_t registry_fingerprint, const MessageEntry *messages,
           std::size_t message_count, const ProfileMetadata *profiles,
           std::size_t profile_count)
      : provider_(provider_definition_id), fingerprint_(registry_fingerprint),
        messages_(messages), message_count_(message_count),
        profiles_(profiles), profile_count_(profile_count) {}

  const char *providerDefinitionId() const { return provider_; }
  std::uint64_t registryFingerprint() const { return fingerprint_; }
  // The first profile (physical VRPN); every profile of a provider shares
  // the parameter and operation vocabulary the runtimes validate against it.
  const ProfileMetadata &primaryProfile() const { return profiles_[0]; }
  const ProfileMetadata *profiles(std::size_t *count) const;

  const ProfileMetadata *profile(const std::string &profile_id) const;
  const char *profileDigest(const std::string &profile_id) const;
  const ParameterMetadata *profileParameters(const std::string &profile_id,
                                             std::size_t *count) const;
  bool parameterMetadata(const std::string &profile_id, const std::string &name,
                         ParameterMetadata *out) const;
  const OperationMetadata *profileOperations(const std::string &profile_id,
                                             std::size_t *count) const;
  bool operationMetadata(const std::string &profile_id,
                         const std::string &operation_id,
                         OperationMetadata *out) const;
  const ChannelMetadata *profileChannels(const std::string &profile_id,
                                         std::size_t *count) const;
  bool channelMetadata(const std::string &profile_id,
                       const std::string &channel_id,
                       ChannelMetadata *out) const;
  bool messageMetadata(std::uint32_t id, MessageMetadata *out) const;

private:
  const char *provider_;
  std::uint64_t fingerprint_;
  const MessageEntry *messages_;
  std::size_t message_count_;
  const ProfileMetadata *profiles_;
  std::size_t profile_count_;
};

// True when the channel is an operation that releases, holds or stops.
inline bool SafetyOperation(const ChannelMetadata &channel) {
  const std::string safety(channel.operation_contract.safety);
  return safety == "release" || safety == "hold" || safety == "stop";
}

const EndpointMetadata *channelEndpoint(const ChannelMetadata &channel,
                                        EndpointKind kind,
                                        const std::string &role);
bool channelObserves(const ChannelMetadata &channel,
                     const std::string &channel_id);
const PolicyMetadata *channelPolicy(const ChannelMetadata &channel,
                                    const std::string &key);
bool channelPolicyBoolean(const ChannelMetadata &channel, const std::string &key,
                          bool *out);
bool channelPolicyInteger(const ChannelMetadata &channel, const std::string &key,
                          std::int64_t *out);
bool channelPolicyNumber(const ChannelMetadata &channel, const std::string &key,
                         double *out);
bool channelPolicyString(const ChannelMetadata &channel, const std::string &key,
                         const char **out);
bool channelPolicyStringArray(const ChannelMetadata &channel,
                              const std::string &key,
                              const char *const **out, std::size_t *count);

} // namespace contract
} // namespace xgc2_ros1_robot_adapter
