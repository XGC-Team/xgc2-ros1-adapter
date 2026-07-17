#include "xgc2_ros1_robot_adapter/runtime_support.hpp"

#include <set>
#include <utility>

namespace xgc2_ros1_robot_adapter {
namespace {

bool fail(std::string *error, const std::string &message) {
  if (error != nullptr)
    *error = message;
  return false;
}

} // namespace

bool BootstrapFileFromArguments(int argc, char **argv, std::string *path,
                                std::string *error) {
  if (path == nullptr)
    return fail(error, "bootstrap output must not be null");
  std::string selected;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index] == nullptr ? "" : argv[index]);
    if (argument == "--adapter-bootstrap-file") {
      if (index + 1 >= argc || argv[index + 1] == nullptr ||
          std::string(argv[index + 1]).empty()) {
        return fail(error, "--adapter-bootstrap-file requires a path");
      }
      if (!selected.empty())
        return fail(error, "--adapter-bootstrap-file was repeated");
      selected = argv[++index];
      continue;
    }
    const std::string prefix = "--adapter-bootstrap-file=";
    if (argument.compare(0, prefix.size(), prefix) == 0) {
      if (!selected.empty())
        return fail(error, "--adapter-bootstrap-file was repeated");
      selected = argument.substr(prefix.size());
      if (selected.empty())
        return fail(error, "--adapter-bootstrap-file requires a path");
    }
  }
  if (selected.empty())
    return fail(error, "trusted --adapter-bootstrap-file is required");
  *path = std::move(selected);
  if (error != nullptr)
    error->clear();
  return true;
}

bool BindBootstrapCapability(
    xgc2::adapter_runtime::ClientConfig *config,
    const std::string &capability_id,
    xgc2::adapter_runtime::CapabilityCallbacks callbacks,
    std::string *error) {
  if (config == nullptr)
    return fail(error, "Adapter Runtime client configuration is required");
  const auto &contracts = config->registration().supported_capabilities();
  const xgc::adapter::v1::CapabilityContract *selected = nullptr;
  for (const auto &contract : contracts) {
    if (contract.capability_id() != capability_id ||
        contract.contract_version() != kRobotCapabilityVersion)
      continue;
    if (selected != nullptr)
      return fail(error, "bootstrap repeats capability " + capability_id);
    selected = &contract;
  }
  if (selected == nullptr)
    return fail(error, "bootstrap does not grant capability " + capability_id);
  return config->BindCapability(capability_id, kRobotCapabilityVersion,
                                selected->contract_digest(),
                                std::move(callbacks), error);
}

bool ResolveRobotSubject(const xgc::adapter::v1::WorkContext &context,
                         const RobotAdapterConfig &configuration,
                         std::string *robot_id, std::string *error) {
  if (robot_id == nullptr)
    return fail(error, "robot subject output must not be null");
  if (!context.has_subject())
    return fail(error, "robot operation is missing its subject");
  const auto &subject = context.subject();
  if (subject.kind() != "robot-resource" || subject.key().empty() ||
      subject.attributes_size() != 3) {
    return fail(error,
                "robot subject must be robot-resource with exactly target-id, "
                "run-id, and robot-id");
  }
  const auto target = subject.attributes().find("target-id");
  const auto run = subject.attributes().find("run-id");
  const auto robot = subject.attributes().find("robot-id");
  if (target == subject.attributes().end() ||
      run == subject.attributes().end() ||
      robot == subject.attributes().end() || robot->second.empty()) {
    return fail(error, "robot subject attributes are incomplete");
  }
  const auto expected_target = configuration.scope_attributes.find("target-id");
  const auto expected_run = configuration.scope_attributes.find("run-id");
  if (expected_target == configuration.scope_attributes.end() ||
      expected_run == configuration.scope_attributes.end() ||
      target->second != expected_target->second || run->second != expected_run->second) {
    return fail(error, "robot subject crosses the applied target/run scope");
  }
  bool found = false;
  for (const auto &candidate : configuration.robots) {
    if (candidate.robot_id == robot->second) {
      found = true;
      break;
    }
  }
  if (!found)
    return fail(error, "robot subject is not present in the applied instance spec");
  *robot_id = robot->second;
  if (error != nullptr)
    error->clear();
  return true;
}

xgc2::adapter_runtime::OperationResult
EmptyOperationSuccess(std::uint32_t schema_version,
                      std::uint64_t schema_fingerprint) {
  xgc::v1::Empty empty;
  xgc::v1::Payload output;
  output.mutable_schema()->set_message_id(1u);
  output.mutable_schema()->set_type_name("xgc.v1.Empty");
  output.mutable_schema()->set_schema_version(schema_version);
  output.mutable_schema()->set_schema_fingerprint(schema_fingerprint);
  output.set_encoding(xgc::v1::PAYLOAD_ENCODING_PROTOBUF);
  output.set_value(empty.SerializeAsString());
  return xgc2::adapter_runtime::OperationResult::Success(std::move(output),
                                                          true);
}

} // namespace xgc2_ros1_robot_adapter
