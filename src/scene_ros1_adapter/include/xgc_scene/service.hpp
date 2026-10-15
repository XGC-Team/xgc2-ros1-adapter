#pragma once

#include "xgc_scene/store.hpp"
#include "xgc_scene/command_dispatch.hpp"

#include <xgc2/xrpc/http.hpp>

#include <functional>
#include <memory>
#include <optional>
#include <string>

namespace xgc_scene {

// The service needs no ROS handle. SceneNode and the wire replay fake both
// provide this seam; retained results always come from the store.
class AuthoringNode {
 public:
  virtual ~AuthoringNode() = default;
  virtual bool online() const = 0;
  virtual Json envelope() = 0;
  virtual Json commandValue(const Json& command, CommandContext context = {}) = 0;
  virtual std::optional<Json> requestResult(const std::string& request_id) = 0;
};

class SceneService {
 public:
  using Complete = std::function<void(xgc2::xrpc::HttpResponse)>;
  using Cancelled = std::function<bool()>;

  SceneService(std::string path, std::string target_id, std::string profile);
  ~SceneService();
  SceneService(const SceneService&) = delete;
  SceneService& operator=(const SceneService&) = delete;

  const Json& reference() const;
  static xgc2::xrpc::HttpLimits limits();
  // Call once after the store, ROS outputs and listening sockets are ready.
  // facts.profile, when present, is the resolved private ~profile parameter.
  void ready(std::shared_ptr<AuthoringNode> node, Json facts = Json::object());
  void facts(Json facts, std::function<Json()> current = {});
  void stopping();
  // Completes discovery asynchronously at readiness or its deadline. Business
  // work runs off the HTTP owner; the host retains admission through Complete.
  void handle(xgc2::xrpc::HttpRequest request, Complete complete, Cancelled cancelled = {});
  void handle(xgc2::xrpc::HttpRequest request, xgc2::xrpc::HttpReply reply);
  void close();

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace xgc_scene
