#pragma once

#include <functional>
#include <mutex>
#include <string>

#include <zenoh.h>

namespace xgc_mocap_rotor_ros1_adapter {

class ZenohSubscriber {
public:
  using Handler = std::function<void(std::string, std::string)>;

  explicit ZenohSubscriber(Handler handler);
  ~ZenohSubscriber();

  ZenohSubscriber(const ZenohSubscriber &) = delete;
  ZenohSubscriber &operator=(const ZenohSubscriber &) = delete;

  bool Start(const std::string &listen_endpoint, std::string *error);
  void Stop();
  bool running() const;

private:
  static void HandleSample(z_loaned_sample_t *sample, void *context);
  void onSample(z_loaned_sample_t *sample) noexcept;

  const Handler handler_;
  mutable std::mutex mutex_;
  bool running_ = false;
  z_owned_session_t session_;
  z_owned_subscriber_t subscriber_;
};

} // namespace xgc_mocap_rotor_ros1_adapter
