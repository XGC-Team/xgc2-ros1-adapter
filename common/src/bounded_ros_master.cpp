// roscpp calls master::execute through its public ELF symbol. These three
// executables export this bounded implementation, including roscpp's own calls.
// Only management uses it; ROS topic/service wire interfaces are unchanged.
#include "xgc2_ros1_robot_adapter/robot_server.hpp"
#include <ros/master.h>
#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcValue.h>

namespace {
thread_local std::chrono::steady_clock::time_point master_deadline;
thread_local bool master_failed = false;
}
namespace xgc2_ros1_robot_adapter {
void ConfigureBoundedRosMaster() {}
RosMasterDeadline::RosMasterDeadline(std::chrono::milliseconds budget)
    : previous_(master_deadline), previous_failed_(master_failed) {
  master_deadline = std::chrono::steady_clock::now() + budget;
  master_failed = false;
}
RosMasterDeadline::~RosMasterDeadline() { master_deadline = previous_; master_failed = previous_failed_; }
bool RosMasterDeadline::failed() const { return master_failed; }
}
namespace ros { namespace master {
bool execute(const std::string &method, const XmlRpc::XmlRpcValue &request,
             XmlRpc::XmlRpcValue &response, XmlRpc::XmlRpcValue &payload, bool) {
  using Services = xgc2_ros1_robot_adapter::AsyncRosServices;
  thread_local Services services;
  Services::Request call;
  call.master_uri = getURI();
  call.deadline = Services::Clock::now() + std::chrono::seconds(3);
  if (master_deadline != Services::Clock::time_point{}) call.deadline = std::min(call.deadline, master_deadline);
  if (call.deadline <= Services::Clock::now()) { master_failed = true; return false; }
  call.master_xml = "<?xml version=\"1.0\"?><methodCall><methodName>" + method + "</methodName><params>";
  for (int i = 0; i < request.size(); ++i) call.master_xml += "<param>" + request[i].toXml() + "</param>";
  call.master_xml += "</params></methodCall>";
  bool success = false, responsive = false;
  services.Start(std::move(call), [&](Services::Result result) {
    if (result.outcome != Services::Outcome::Response) return;
    const auto start = result.response.find("<value>");
    if (start == std::string::npos) return;
    int offset = start;
    try {
      if (response.fromXml(result.response, &offset) && response.getType() == XmlRpc::XmlRpcValue::TypeArray &&
          response.size() == 3 && response[0].getType() == XmlRpc::XmlRpcValue::TypeInt) {
        responsive = true;
        if (int(response[0]) == 1) { payload = response[2]; success = true; }
      }
    } catch (...) { success = false; }
  });
  while (services.pending()) services.Pump(std::chrono::milliseconds(10));
  master_failed = master_failed || !responsive;
  return success;
}
} }
