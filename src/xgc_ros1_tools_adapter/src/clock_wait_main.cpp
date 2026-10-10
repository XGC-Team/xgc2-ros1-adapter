#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#include "xgc_ros1_tools_adapter/clock_wait.hpp"

namespace {
volatile std::sig_atomic_t stop_requested = 0;
void requestStop(int) { stop_requested = 1; }
}  // namespace

int main(int argc, char** argv) {
  try {
    std::string target, topic = "/clock";
    unsigned timeout = 300;
    // ROS remapping arguments are handled by ros::init after explicit options.
    ros::V_string arguments;
    ros::removeROSArgs(argc, argv, arguments);
    for (std::size_t i = 1; i < arguments.size(); i += 2) {
      if (i + 1 == arguments.size())
        throw std::invalid_argument("option requires a value");
      const auto& key = arguments[i];
      const auto& value = arguments[i + 1];
      if (key == "--target-seconds" && target.empty()) {
        target = value;
      } else if (key == "--clock-topic") {
        topic = value;
      } else if (key == "--timeout-seconds") {
        if (value.empty() ||
            value.find_first_not_of("0123456789") != std::string::npos) {
          throw std::invalid_argument("timeout must be integer seconds");
        }
        const auto parsed = std::stoul(value);
        if (parsed < 1 || parsed > 86400)
          throw std::invalid_argument("timeout must be 1..86400 seconds");
        timeout = static_cast<unsigned>(parsed);
      } else {
        throw std::invalid_argument("unknown or repeated option: " + key);
      }
    }
    std::string reason;
    if (topic.empty() || topic.front() != '/' ||
        !ros::names::validate(topic, reason)) {
      throw std::invalid_argument("clock topic must be an absolute ROS name");
    }
    xgc_ros1_tools_adapter::ClockWait wait(target);
    std::signal(SIGINT, requestStop);
    std::signal(SIGTERM, requestStop);
    ros::init(
        argc, argv, "xgc_clock_wait",
        ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
    ros::NodeHandle node;
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(timeout);
    bool reached = false;
    std::string failure;
    ros::Time stamp;
    auto subscription = node.subscribe<rosgraph_msgs::Clock>(
        topic, 256, [&](const rosgraph_msgs::Clock::ConstPtr& message) {
          if (reached || !failure.empty()) return;
          try {
            if (std::chrono::steady_clock::now() >= deadline)
              throw std::runtime_error("simulation wait timed out");
            reached = wait.observe(message->clock.sec, message->clock.nsec);
            stamp = message->clock;
          } catch (const std::exception& error) {
            failure = error.what();
          }
        });
    while (!stop_requested && ros::ok() && !reached && failure.empty()) {
      const auto publishers = subscription.getNumPublishers();
      if (stop_requested || !ros::ok()) return 130;
      if (publishers > 1)
        throw std::runtime_error("clock must have exactly one publisher");
      if (wait.observed() && publishers == 0)
        throw std::runtime_error("clock publisher disconnected");
      if (std::chrono::steady_clock::now() >= deadline)
        throw std::runtime_error("simulation wait timed out");
      ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.05));
    }
    if (stop_requested) return 130;
    if (!failure.empty()) throw std::runtime_error(failure);
    if (!reached) return 130;
    if (subscription.getNumPublishers() > 1)
      throw std::runtime_error("clock must have exactly one publisher");
    std::cout << "{\"clock\":{\"sec\":" << stamp.sec
              << ",\"nsec\":" << stamp.nsec
              << "},\"reachedSeconds\":" << stamp.sec << '.' << std::setw(9)
              << std::setfill('0') << stamp.nsec << "}\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
