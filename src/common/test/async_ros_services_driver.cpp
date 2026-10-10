#include "xgc2_ros1_robot_adapter/async_ros_services.hpp"
#include <chrono>
#include <iostream>
#include <string>

int main(int argc, char **argv) {
  if (argc < 4 || argc > 7) return 2;
  using Services = xgc2_ros1_robot_adapter::AsyncRosServices;
  Services client(argc > 4 ? argv[4] : "");
  std::uint64_t first = 0;
  const auto cancel_at = Services::Clock::now() + std::chrono::milliseconds(argc > 5 ? std::stoi(argv[5]) : 0);
  const int count = std::stoi(argv[2]);
  const int timeout_ms = std::stoi(argv[3]);
  const bool master = argc == 7 && std::string(argv[6]) == "master";
  for (int i = 0; i != count; ++i) {
    Services::Request request;
    request.master_uri = argv[1]; request.caller_id = "/async_transport_test";
    request.service = "/robot" + std::to_string(i) + "/arm";
    request.type = "mavros_msgs/CommandBool";
    request.md5 = "e09abbb4e5bae6b558e501096e7eb71e";
    request.bytes = std::string(1, '\1');
    if (master) request.master_xml = "<methodCall><methodName>getParam</methodName><params></params></methodCall>";
    request.deadline = Services::Clock::now() + std::chrono::milliseconds(timeout_ms);
    auto id = client.Start(std::move(request), [i](Services::Result result) {
      std::cout << i << " " << int(result.outcome) << " " << result.response.size() << std::endl;
    });
    if (!first) first = id;
    if (master) while (client.pending()) client.Pump(std::chrono::milliseconds(2));
  }
  while (client.pending()) {
    if (argc > 5 && first && Services::Clock::now() >= cancel_at) { client.Cancel(first); first = 0; }
    client.Pump(std::chrono::milliseconds(2));
  }
}
