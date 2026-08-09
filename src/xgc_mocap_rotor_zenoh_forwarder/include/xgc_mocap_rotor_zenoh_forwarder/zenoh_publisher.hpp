#pragma once

#include <mutex>
#include <string>

#include <zenoh.h>

#include "xgc_mocap_rotor_zenoh_forwarder/forwarder_contract.hpp"

namespace xgc_mocap_rotor_zenoh_forwarder {

class ZenohPublisher {
public:
  ZenohPublisher();
  ~ZenohPublisher();

  ZenohPublisher(const ZenohPublisher &) = delete;
  ZenohPublisher &operator=(const ZenohPublisher &) = delete;

  bool Start(const std::string &robot_id, const std::string &connect_endpoint,
             std::string *error);
  bool Put(UplinkChannel channel, const std::string &payload,
           std::string *error);
  void Stop();
  bool running() const;

private:
  mutable std::mutex mutex_;
  z_owned_session_t session_;
  std::string robot_id_;
  bool running_ = false;
};

} // namespace xgc_mocap_rotor_zenoh_forwarder
