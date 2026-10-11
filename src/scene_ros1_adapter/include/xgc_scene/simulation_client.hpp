#pragma once

#include "xgc_scene/common.hpp"

#include <xgc2/xrpc/http.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

namespace xgc_scene {

class Simulation {
 public:
  virtual ~Simulation() = default;
  // Latest desired state only. Neither method waits for a peer or calls ROS.
  virtual void update(const Json& document, const std::string& epoch, std::uint64_t revision,
                      const Json& motion) = 0;
  virtual std::optional<Json> takeReport() = 0;
  virtual void close() = 0;
};

class SimulationClient final : public Simulation {
 public:
  SimulationClient(const Json& reference, const std::string& local_target,
                   std::chrono::milliseconds timeout = std::chrono::seconds(30));
  ~SimulationClient() override;
  void update(const Json& document, const std::string& epoch, std::uint64_t revision,
              const Json& motion) override;
  std::optional<Json> takeReport() override;
  void close() override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace xgc_scene
