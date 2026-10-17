#include "xgc2_ros1_robot_adapter/async_ros_services.hpp"
#include "xgc2_ros1_robot_adapter/ros1_binding.hpp"
#include <ros/master.h>
#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <chrono>
#include <iostream>
#include <string>

int main(int argc, char **argv) {
  if (argc < 4 || argc > 7) return 2;
  using Services = xgc2_ros1_robot_adapter::AsyncRosServices;
  const std::string mode = argc == 7 ? argv[6] : "";
  if (mode == "registration-tracking" || mode == "registration-cumulative") {
    ros::M_string remappings;
    remappings["__master"] = argv[1];
    ros::init(remappings, "registration_transport_test",
              ros::init_options::NoSigintHandler);
    const auto duration = std::chrono::milliseconds(std::stoi(argv[3]));
    xgc2_ros1_robot_adapter::RosMasterDeadline budget(
        mode == "registration-tracking"
            ? std::nullopt
            : std::optional<std::chrono::milliseconds>(duration));
    {
      xgc2_ros1_robot_adapter::RosMasterDeadline nested(std::nullopt);
      for (int i = 0; i != std::stoi(argv[2]); ++i) {
        XmlRpc::XmlRpcValue request, response, payload;
        request[0] = "/registration_transport_test";
        request[1] = "/robot" + std::to_string(i) + "/pose";
        request[2] = "geometry_msgs/PoseStamped";
        request[3] = "http://127.0.0.1:1/";
        const bool ok = ros::master::execute("registerSubscriber", request,
                                             response, payload, false);
        std::cout << "register " << i << " " << ok << " " << budget.failed()
                  << std::endl;
        if (!ok) break; // A native factory rolls back on the first rejected handle.
      }
    }
    XmlRpc::XmlRpcValue request, response, payload;
    request[0] = "/registration_transport_test";
    request[1] = "/robot0/pose";
    request[2] = "http://127.0.0.1:1/";
    const bool removed = ros::master::execute("unregisterSubscriber", request,
                                              response, payload, false);
    std::cout << "rollback " << removed << " " << budget.failed() << std::endl;
    return 0;
  }
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
