#pragma once

#include "xgc_scene/ros_outputs.hpp"
#include "xgc_scene/service.hpp"
#include "xgc_scene/simulation_client.hpp"

#include <atomic>
#include <filesystem>
#include <mutex>
#include <optional>
#include <unordered_map>

namespace xgc_scene {

struct SceneParameters {
  std::filesystem::path scene_file, working_file, save_directory;
  Json generation_parameters = Json::object();
  std::string target_id, xrpc_socket, profile = "gazebo", simulation_endpoint;
  bool frozen = false;
  static SceneParameters parse(const Json& private_parameters);
};

class SceneNode final : public AuthoringNode {
 public:
  using Clock = SceneStore::Clock;
  SceneNode(const SceneParameters& parameters, const std::string& name_space,
            RosOutputs& outputs, Clock clock, std::shared_ptr<Simulation> simulation = {});
  // In-memory construction uses the same store/hooks/projections as startup.
  SceneNode(const Json& initial, const SceneParameters& parameters, const std::string& name_space,
            RosOutputs& outputs, Clock clock, std::shared_ptr<Simulation> simulation = {},
            std::optional<std::string> source_digest = {});
  ~SceneNode() override;
  bool online() const override;
  Json envelope() override;
  Json commandValue(const Json& command, CommandContext context = {}) override;
  std::optional<Json> requestResult(const std::string& request_id) override;
  Json applicationView(const std::string& epoch, std::uint64_t revision);
  Json peerFacts();
  void gazeboFailure(const std::string& epoch, std::uint64_t revision, const std::string& message);
  void publishDefinition();
  bool publishDocument();
  void tick();
  void shutdown();
  // Cancels an in-progress native call before shutdown takes the node lock.
  void cancelSimulation();
  SceneProjection::Counters counters();

 private:
  void initialize(const Json& initial, const std::optional<std::string>& source_digest);
  void applyScene(const Json& document, const std::string& epoch, std::uint64_t revision);
  void applyMotion(const std::string& operation, const std::string& epoch, std::uint64_t revision);
  void queueSimulation();
  void receiveSimulation();
  void addPublicView(Json& envelope);

  // Lane 0 owns store hooks and publications. Peer IO never holds this lock.
  std::recursive_mutex mutex_;
  SceneParameters parameters_;
  RosOutputs& outputs_;
  Clock clock_;
  std::shared_ptr<Simulation> simulation_;
  std::unique_ptr<SceneStore> store_;
  SceneProjection projection_;
  EnvelopeJson encoder_;
  std::atomic<bool> online_{true};
  std::optional<Json> native_report_, published_view_;
  std::optional<xgc2_geometry_msgs::SceneSnapshot> applied_snapshot_;
  Json definition_;
  std::string motion_operation_;
  // Store receipts contain core fields. These bounded public projections keep
  // native reports and online state at receipt time, including on replay.
  std::unordered_map<std::string, Json> public_receipts_;
};

}  // namespace xgc_scene
