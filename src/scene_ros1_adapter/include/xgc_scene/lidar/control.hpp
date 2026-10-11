#pragma once

#include <functional>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>

namespace xgc_scene::lidar {

// One ROS-free control host for the fleet. post must enqueue callbacks on the
// executor's control thread; apply runs there and must return in finite time.
class Control {
 public:
  Control(std::string socket, std::string target, bool initial_enabled,
          std::function<void(std::function<void()>)> post,
          std::function<void(bool)> apply,
          nlohmann::json facts = nlohmann::json::object());
  ~Control();
  Control(const Control&) = delete;
  Control& operator=(const Control&) = delete;

  void Start();
  // Stop joins the HTTP owner and waits for any active apply call. Queued
  // executor callbacks become inert and may safely run after destruction.
  void Stop();
  // The service is ready on listening by default. An owner with an additional
  // dependency can hold discovery until it has made that dependency ready.
  void SetReady(bool ready);
  void SetFacts(nlohmann::json facts, std::function<nlohmann::json()> current = {});

 private:
  class Impl;
  std::shared_ptr<Impl> impl_;
};

}  // namespace xgc_scene::lidar
