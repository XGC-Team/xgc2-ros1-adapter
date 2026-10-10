// Command-line client for the robot type server gRPC service. It exists for
// the integration tests (Python has no gRPC here) and for manual diagnosis: one
// invocation sends one RPC and prints the result as JSON on stdout.
//
//   robot_server_client --socket PATH [--deadline-ms N] COMMAND [ARGS]
//
//   describe [WAIT_MS]    Describe, holding up to WAIT_MS for readiness
//   health                Health
//   apply                 ApplyMembers; request JSON on stdin
//   remove                RemoveMembers; request JSON on stdin
//   execute               Execute; request JSON on stdin
//   watch SECONDS         SubscribeStatus; one response JSON per line
//   shutdown              Shutdown
//
// Every command except describe first discovers the instance with Describe and
// binds the call to it, as Core does.
#include "xgc/robot/v1/server.grpc.pb.h"
#include "xgc2/xrpc/grpc.hpp"
#include <chrono>
#include <google/protobuf/util/json_util.h>
#include <iostream>
#include <iterator>
#include <nlohmann/json.hpp>
#include <string>

namespace {
namespace wire = xgc::robot::v1;
using xgc2::xrpc::GrpcClientCall;
using xgc2::xrpc::GrpcClock;

std::string toJson(const google::protobuf::Message &message) {
  google::protobuf::util::JsonPrintOptions options;
  options.preserve_proto_field_names = true;
  options.always_print_primitive_fields = true;
  std::string text;
  const auto status =
      google::protobuf::util::MessageToJsonString(message, &text, options);
  if (!status.ok())
    throw std::runtime_error("cannot print response: " + status.ToString());
  return text;
}
template <class Message> Message fromJson(const std::string &text) {
  Message message;
  google::protobuf::util::JsonParseOptions options;
  const auto status =
      google::protobuf::util::JsonStringToMessage(text, &message, options);
  if (!status.ok())
    throw std::runtime_error("invalid request JSON: " + status.ToString());
  return message;
}
std::string readAll(std::istream &stream) {
  return {std::istreambuf_iterator<char>(stream),
          std::istreambuf_iterator<char>()};
}
const char *deliveryName(xgc2::xrpc::Delivery delivery) {
  return xgc2::xrpc::delivery_name(delivery);
}
void print(const grpc::Status &status, xgc2::xrpc::Delivery delivery,
           const std::string &response_json) {
  nlohmann::json result;
  result["status"] = {{"code", static_cast<int>(status.error_code())},
                      {"message", status.error_message()}};
  result["delivery"] = deliveryName(delivery);
  result["response"] = response_json.empty()
                           ? nlohmann::json(nullptr)
                           : nlohmann::json::parse(response_json);
  std::cout << result.dump() << std::endl;
}

class Client {
public:
  Client(const std::string &socket, std::chrono::milliseconds deadline)
      : channel_(xgc2::xrpc::make_grpc_unix_channel(socket)),
        stub_(wire::RobotAdapterServerService::NewStub(channel_)),
        budget_(deadline) {}

  int describe(std::uint32_t wait_ms) {
    grpc::ClientContext context;
    GrpcClientCall call(context, "", GrpcClock::now() + budget_, {}, "", true);
    wire::DescribeRequest request;
    request.set_wait_ready_ms(wait_ms);
    wire::DescribeResponse response;
    const auto status = call.invoke(
        [&] { return stub_->Describe(&context, request, &response); });
    print(status, call.delivery(), status.ok() ? toJson(response) : "");
    return status.ok() ? 0 : 1;
  }
  std::string discover() {
    grpc::ClientContext context;
    GrpcClientCall call(context, "", GrpcClock::now() + budget_, {}, "", true);
    wire::DescribeRequest request;
    wire::DescribeResponse response;
    const auto status = call.invoke(
        [&] { return stub_->Describe(&context, request, &response); });
    if (!status.ok())
      throw std::runtime_error("discovery failed: " + status.error_message());
    return call.response_instance_id();
  }
  template <class Request, class Response, class Method>
  int unary(const Request &request, Method method) {
    const auto instance = discover();
    grpc::ClientContext context;
    GrpcClientCall call(context, instance, GrpcClock::now() + budget_);
    Response response;
    const auto status = call.invoke(
        [&] { return (stub_.get()->*method)(&context, request, &response); });
    print(status, call.delivery(), status.ok() ? toJson(response) : "");
    return status.ok() ? 0 : 1;
  }
  int watch(std::chrono::seconds duration) {
    const auto instance = discover();
    grpc::ClientContext context;
    GrpcClientCall call(context, instance,
                        xgc2::xrpc::grpc_stream_deadline(
                            xgc2::xrpc::GrpcLimits{},
                            GrpcClock::now() + duration));
    wire::SubscribeStatusRequest request;
    const auto mark = call.mark_dispatched();
    if (!mark.ok())
      return 1;
    auto stream = stub_->SubscribeStatus(&context, request);
    const auto identity = call.receive_initial_metadata(*stream);
    if (!identity.ok()) {
      print(identity, call.delivery(), "");
      return 1;
    }
    const auto end = GrpcClock::now() + duration;
    wire::SubscribeStatusResponse response;
    while (GrpcClock::now() < end && stream->Read(&response)) {
      std::cout << toJson(response) << std::endl;
    }
    context.TryCancel();
    stream->Finish();
    return 0;
  }

private:
  std::shared_ptr<grpc::Channel> channel_;
  std::unique_ptr<wire::RobotAdapterServerService::Stub> stub_;
  std::chrono::milliseconds budget_;
};
} // namespace

int main(int argc, char **argv) {
  std::string socket;
  std::chrono::milliseconds deadline(10000);
  int index = 1;
  for (; index + 1 < argc && std::string(argv[index]).rfind("--", 0) == 0;
       index += 2) {
    const std::string flag = argv[index];
    if (flag == "--socket")
      socket = argv[index + 1];
    else if (flag == "--deadline-ms")
      deadline = std::chrono::milliseconds(std::stol(argv[index + 1]));
    else {
      std::cerr << "unknown flag " << flag << std::endl;
      return 2;
    }
  }
  if (socket.empty() || index >= argc) {
    std::cerr << "usage: robot_server_client --socket PATH [--deadline-ms N] "
                 "describe [WAIT_MS]|health|apply|remove|execute|watch "
                 "SECONDS|shutdown\n";
    return 2;
  }
  const std::string command = argv[index];
  try {
    Client client(socket, deadline);
    if (command == "describe")
      return client.describe(index + 1 < argc ? std::stoul(argv[index + 1]) : 0);
    if (command == "health")
      return client.unary<wire::HealthRequest, wire::HealthResponse>(
          wire::HealthRequest(), &wire::RobotAdapterServerService::Stub::Health);
    if (command == "apply")
      return client.unary<wire::ApplyMembersRequest, wire::ApplyMembersResponse>(
          fromJson<wire::ApplyMembersRequest>(readAll(std::cin)),
          &wire::RobotAdapterServerService::Stub::ApplyMembers);
    if (command == "remove")
      return client.unary<wire::RemoveMembersRequest, wire::RemoveMembersResponse>(
          fromJson<wire::RemoveMembersRequest>(readAll(std::cin)),
          &wire::RobotAdapterServerService::Stub::RemoveMembers);
    if (command == "execute")
      return client.unary<wire::ExecuteRequest, wire::ExecuteResponse>(
          fromJson<wire::ExecuteRequest>(readAll(std::cin)),
          &wire::RobotAdapterServerService::Stub::Execute);
    if (command == "shutdown")
      return client.unary<wire::ShutdownRequest, wire::ShutdownResponse>(
          wire::ShutdownRequest(), &wire::RobotAdapterServerService::Stub::Shutdown);
    if (command == "watch" && index + 1 < argc)
      return client.watch(std::chrono::seconds(std::stol(argv[index + 1])));
    std::cerr << "unknown command " << command << std::endl;
    return 2;
  } catch (const std::exception &error) {
    std::cerr << "robot_server_client: " << error.what() << std::endl;
    return 3;
  }
}
