#include "xgc2_ros1_robot_adapter/ros1_binding.hpp"
#include <csignal>
#include <iostream>

namespace xgc2_ros1_robot_adapter {
namespace {
RobotServer *running_server = nullptr;
// The server drains and exits from its own thread; the handler only asks.
void onStopSignal(int) {
  if (running_server != nullptr)
    running_server->RequestStop();
}
} // namespace

int RunRos1TypeServer(int argc, char **argv,
                      const std::vector<Ros1Provider> &providers) {
  std::vector<std::string> names;
  for (const auto &entry : providers)
    names.push_back(entry.provider);
  ServerOptions options;
  try {
    options = ParseServerOptions(argc, argv, names);
  } catch (const std::exception &error) {
    std::cerr << "robot server: " << error.what() << '\n'
              << ServerUsage(argv[0]) << std::endl;
    return kExitUsage;
  }
  try {
    RobotFactory factory;
    for (const auto &entry : providers)
      if (entry.provider == options.provider)
        factory = entry.factory;
    Ros1Binding binding;
    RobotServer server(options, binding, factory);
    running_server = &server;
    struct sigaction action {};
    action.sa_handler = onStopSignal;
    sigemptyset(&action.sa_mask);
    sigaction(SIGINT, &action, nullptr);
    sigaction(SIGTERM, &action, nullptr);
    const int code = server.Run();
    running_server = nullptr;
    return code;
  } catch (const std::exception &error) {
    std::cerr << "robot server: " << error.what() << std::endl;
    return kExitFailure;
  }
}
} // namespace xgc2_ros1_robot_adapter
