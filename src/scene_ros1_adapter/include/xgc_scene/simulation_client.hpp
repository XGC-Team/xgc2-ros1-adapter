#pragma once

#include "xgc_scene/common.hpp"

#include <xgc2/xrpc/http.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

namespace xgc_scene {

class Simulation {
 public:
  virtual ~Simulation() = default;
  virtual Json apply(const Json& document, const std::string& epoch, std::uint64_t revision) = 0;
  virtual Json motion(const std::string& operation, const std::string& epoch, std::uint64_t revision) = 0;
  virtual void close() = 0;
};

class SimulationClient final : public Simulation {
 public:
  SimulationClient(const Json& reference, const std::string& local_target,
                   std::chrono::milliseconds timeout = std::chrono::seconds(30));
  ~SimulationClient() override;
  Json operation(const std::string& route, const Json& value, const std::string& method = "POST",
                 std::string request_id = {});
  Json apply(const Json& document, const std::string& epoch, std::uint64_t revision) override;
  Json motion(const std::string& operation, const std::string& epoch, std::uint64_t revision) override;
  void close() override;

 private:
  Json call(const std::string& route, const Json& value, const std::string& method,
            const std::string& request_id, xgc2::xrpc::Clock::time_point deadline);
  std::chrono::milliseconds timeout_;
  std::unique_ptr<xgc2::xrpc::HttpClient> client_;
};

}  // namespace xgc_scene
