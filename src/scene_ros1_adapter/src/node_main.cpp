#include "xgc_scene/node.hpp"
#include "xgc_scene/prepare.hpp"
#include "xgc_scene/json_io.hpp"
#include "xgc_scene/fixed_cloud.hpp"
#include "peer_backoff.hpp"
#include <xmlrpcpp/XmlRpcValue.h>
#include "xgc_scene/lidar/control.hpp"
#include "xgc_scene/lidar_ros/edge.hpp"
#include "xgc_scene/lidar_ros/scene_tap.hpp"

#include <xgc2_ros1_robot_adapter/ros1_binding.hpp>
#include <xgc2/xrpc/unix.hpp>
#include <ros/master.h>

#include <atomic>
#include <condition_variable>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <future>
#include <mutex>
#include <thread>

namespace {
namespace adapter = xgc2_ros1_robot_adapter;
using namespace std::chrono;

volatile std::sig_atomic_t signalled = 0;
void signalStop(int) { signalled = 1; }

bool launchBoolean(const std::string& name, const std::string& text) {
  if (text == "true" || text == "True" || text == "TRUE") return true;
  if (text == "false" || text == "False" || text == "FALSE") return false;
  throw xgc_scene::SceneError("Scene parameter ~" + name + " must be a boolean");
}

xgc_scene::Json parameters(ros::NodeHandle& private_node, const adapter::ServerOptions& options) {
  xgc_scene::Json values = xgc_scene::Json::object();
  for (const auto* key : {"scene_file", "working_file", "generation_parameters_json", "save_directory",
                           "target_id", "xrpc_socket", "profile", "simulation_endpoint"}) {
    if (std::string(key) == "simulation_endpoint" && values.value("profile", "gazebo") != "gazebo") continue;
    std::string value;
    if (private_node.getParam(key, value)) values[key] = value;
    else if (private_node.hasParam(key)) throw xgc_scene::SceneError(std::string("Scene parameter ~") + key + " must be a string");
  }
  bool frozen = false;
  if (private_node.getParam("frozen", frozen)) values["frozen"] = frozen;
  else if (private_node.hasParam("frozen")) throw xgc_scene::SceneError("Scene parameter ~frozen must be a boolean");
  if (!values.contains("target_id")) values["target_id"] = options.target_id;
  if (!values.contains("xrpc_socket")) values["xrpc_socket"] = options.socket_path;
  if (values.at("target_id") != options.target_id || values.at("xrpc_socket") != options.socket_path)
    throw xgc_scene::SceneError("Scene private target/socket must match the Run binding");
  return values;
}

struct Objects {
  std::shared_ptr<xgc_scene::RosPublishers> outputs;
  std::shared_ptr<xgc_scene::lidar_ros::SceneTap> tap;
  std::shared_ptr<xgc_scene::lidar_ros::Edge> lidar;
  std::shared_ptr<xgc_scene::SceneNode> node;
  std::shared_ptr<xgc_scene::SimulationClient> simulation;
};

class Dispatch final : public ros::CallbackInterface {
 public:
  explicit Dispatch(std::function<void()> fn) : fn_(std::move(fn)) {}
  CallResult call() override { fn_(); return Success; }
 private:
  std::function<void()> fn_;
};

// The authoring service's business worker waits for its command on lane 0.
// That keeps authored state, native reports, the scene tap, poses and ticks on one owner.
class LaneAuthoring final : public xgc_scene::AuthoringNode {
 public:
  LaneAuthoring(std::shared_ptr<xgc_scene::SceneNode> node,
                std::function<void(std::function<void()>)> post, std::atomic<bool>& stopping)
      : node_(std::move(node)), post_(std::move(post)), stopping_(stopping) {}
  bool online() const override { return node_->online(); }
  xgc_scene::Json envelope() override { return node_->envelope(); }
  std::optional<xgc_scene::Json> requestResult(const std::string& id) override { return node_->requestResult(id); }
  xgc_scene::Json commandValue(const xgc_scene::Json& command) override {
    auto result = std::make_shared<std::promise<xgc_scene::Json>>();
    auto future = result->get_future();
    post_([node = node_, result, command, stopping = &stopping_] {
      try {
        if (stopping->load()) throw xgc_scene::SceneError("Scene process is stopping");
        result->set_value(node->commandValue(command));
      } catch (...) { result->set_exception(std::current_exception()); }
    });
    while (future.wait_for(milliseconds(100)) != std::future_status::ready)
      if (stopping_.load()) throw xgc_scene::SceneError("Scene process is stopping");
    return future.get();
  }
 private:
  std::shared_ptr<xgc_scene::SceneNode> node_;
  std::function<void(std::function<void()>)> post_;
  std::atomic<bool>& stopping_;
};
}  // namespace

int main(int argc, char** argv) {
  adapter::ServerOptions options;
  ros::M_string remappings;
  std::string hostname, profile = "gazebo";
  bool session_stdin = false;
  std::optional<xgc_scene::lidar::SessionInput> session;
  try {
    // Core supplies the same Run/master flags as the other adapters. ROS
    // private parameter and namespace remappings remain native ROS arguments.
    std::vector<std::string> args{argv[0]};
    for (int i = 1; i < argc; ++i) {
      const std::string argument = argv[i];
      if (argument == "--help") {
        std::cout << "usage: xgc2_scene_adapter --socket-path PATH --target-id ID --run-id ID "
                     "--node-name NAME --ros-master-uri URI [--ros-ip IP] [--ros-hostname HOST] [--session-stdin] "
                     "_scene_file:=PATH _lidar_xrpc_socket:=PATH _profile:=gazebo|xsim|physical [ROS private parameters]\n";
        return adapter::kExitOk;
      }
      const auto separator = argument.find(":=");
      if (separator != std::string::npos) {
        const auto key = argument.substr(0, separator), value = argument.substr(separator + 2);
        if (!remappings.emplace(key, value).second) throw std::invalid_argument("duplicate ROS remapping");
        if (key == "_profile") profile = value;
      } else if (argument == "--session-stdin") {
        if (session_stdin) throw std::invalid_argument("--session-stdin was specified twice");
        session_stdin = true;
      } else if (argument == "--ros-hostname") {
        if (++i == argc) throw std::invalid_argument("--ros-hostname requires a value");
        hostname = argv[i];
      } else args.push_back(argument);
    }
    args.insert(args.end(), {"--provider", "xgc2-scene-adapter"});
    std::vector<char*> native;
    for (auto& arg : args) native.push_back(arg.data());
    options = adapter::ParseServerOptions(static_cast<int>(native.size()), native.data(), {"xgc2-scene-adapter"});
    if (profile != "gazebo" && profile != "xsim" && profile != "physical")
      throw xgc_scene::SceneError("Scene profile must be gazebo, xsim or physical");
    if (session_stdin) session = xgc_scene::lidar_ros::readSessionInput(std::cin);
  } catch (const std::exception& error) {
    std::cerr << "scene adapter: " << error.what() << '\n';
    return adapter::kExitUsage;
  }

  std::atomic<bool> stopping{false};
  std::atomic<int> exit_code{adapter::kExitOk};
  std::mutex stop_mutex, objects_mutex;
  std::condition_variable changed;
  Objects objects;
  std::unique_ptr<xgc_scene::SceneService> service;
  std::unique_ptr<xgc_scene::lidar::Control> lidar_control;
  auto stop = [&](int code, const std::string& message) {
    std::lock_guard<std::mutex> lock(stop_mutex);
    if (!stopping.exchange(true)) {
      exit_code.store(code);
      std::cerr << "scene adapter: " << message << '\n';
    }
    changed.notify_all();
  };

  adapter::Ros1Binding binding;
  std::unique_ptr<adapter::RosNode> ros_node;
  std::thread lifecycle;
  std::atomic<ros::CallbackQueueInterface*> lane{nullptr};
  std::atomic<std::uint64_t> master_attempts{0};
  try {
    setenv("ROS_MASTER_URI", options.ros_master_uri.c_str(), 1);
    if (!options.ros_ip.empty()) { setenv("ROS_IP", options.ros_ip.c_str(), 1); unsetenv("ROS_HOSTNAME"); }
    else if (!hostname.empty()) { setenv("ROS_HOSTNAME", hostname.c_str(), 1); unsetenv("ROS_IP"); }
    const auto node_name = adapter::RosNodeName(options.node_name);
    // Install ROS remappings before the common binding's idempotent Init.
    auto transport_remappings = remappings;
    for (auto it = transport_remappings.begin(); it != transport_remappings.end();) {
      if (it->first.size() > 1 && it->first[0] == '_' && it->first[1] != '_') it = transport_remappings.erase(it);
      else ++it;
    }
    ros::init(transport_remappings, node_name, ros::init_options::NoSigintHandler);
    binding.Init(options, node_name);
    ros::master::setRetryTimeout(ros::WallDuration(5));
    std::signal(SIGINT, signalStop);
    std::signal(SIGTERM, signalStop);

    xgc_scene::Json lidar_values = xgc_scene::Json::object();
    for (const auto* key : {"lidar_xrpc_socket", "lidar_defaults_json", "lidar_enabled"}) {
      const auto found = remappings.find(std::string("_") + key);
      if (found == remappings.end()) continue;
      if (std::string(key) == "lidar_enabled") {
        lidar_values[key] = launchBoolean(key, found->second);
      } else lidar_values[key] = found->second;
    }
    auto lidar_parameters = xgc_scene::lidar_ros::Parameters::parse(lidar_values, profile, session);
    if (lidar_parameters.socket == options.socket_path)
      throw xgc_scene::SceneError("Lidar parameter ~lidar_xrpc_socket must differ from ~xrpc_socket");
    auto post = [&lane](std::function<void()> fn) {
      auto* queue = lane.load();
      if (!queue) throw xgc_scene::SceneError("ROS master is waiting");
      queue->addCallback(ros::CallbackInterfacePtr(new Dispatch(std::move(fn))));
    };

    service.reset(new xgc_scene::SceneService(options.socket_path, options.target_id, profile));
    xgc2::xrpc::HttpServer server(xgc2::xrpc::UnixOptions{options.socket_path},
        [&](xgc2::xrpc::HttpRequest request, xgc2::xrpc::HttpReply reply) {
          service->handle(std::move(request), std::move(reply));
        }, xgc_scene::SceneService::limits(),
        {service->reference().at("instance_id").get<std::string>(), {"/v1/describe"}});
    lidar_control = std::make_unique<xgc_scene::lidar::Control>(
        lidar_parameters.socket, options.target_id, lidar_parameters.enabled, post,
        [&](bool enabled) {
          std::shared_ptr<xgc_scene::lidar_ros::Edge> edge;
          { std::lock_guard<std::mutex> lock(objects_mutex); edge = objects.lidar; }
          if (edge) edge->setEnabled(enabled);
        }, lidar_parameters.facts(profile));
    service->facts({{"ros_master", "waiting"}}, [&] { return xgc_scene::Json{{"master_probe_attempts", master_attempts.load()}}; });
    auto waiting_facts = lidar_parameters.facts(profile);
    waiting_facts["ros_master"] = "waiting";
    lidar_control->SetFacts(waiting_facts, [&] { return xgc_scene::Json{{"master_probe_attempts", master_attempts.load()}, {"scene_ready", false}}; });
    lidar_control->SetReady(false);
    lidar_control->Start();

    lifecycle = std::thread([&] {
      std::thread callbacks;
      try {
        adapter::MasterIdentity master;
        xgc_scene::PeerBackoff backoff;
        bool bound = false;
        while (!stopping.load()) {
          XmlRpc::XmlRpcValue request, response, payload;
          request.setSize(1);
          request[0] = "/" + node_name;
          ++master_attempts;
          {
            auto budget = binding.BoundMasterCalls(milliseconds(500));
            bound = ros::master::execute("getPid", request, response, payload, false) &&
                    payload.getType() == XmlRpc::XmlRpcValue::TypeInt;
          }
          if (bound) { backoff.success(); break; }
          const auto delay = backoff.failed("ros_master", "XML-RPC liveness unavailable");
          std::unique_lock<std::mutex> lock(stop_mutex);
          changed.wait_for(lock, delay, [&] { return stopping.load(); });
        }
        if (!bound || stopping.load()) return;
        {
          auto budget = binding.BoundMasterCalls(seconds(5));
          if (!binding.ProbeMaster(&master)) throw xgc_scene::SceneError("ROS master identity unavailable after liveness");
          binding.Connect();
          ros_node = binding.CreateNode(0);
          if (budget->failed()) throw xgc_scene::SceneError("ROS master registration failed");
        }
        auto& handle = adapter::Ros1Handle(*ros_node);
        lane.store(handle.getCallbackQueue());
        ros::NodeHandle private_node("~");
        {
          auto budget = binding.BoundMasterCalls(seconds(5));
          for (const auto& entry : remappings) {
            if (entry.first.size() < 2 || entry.first[0] != '_' || entry.first[1] == '_') continue;
            const auto name = entry.first.substr(1);
            if (name == "frozen" || name == "lidar_enabled") {
              private_node.setParam(name, launchBoolean(name, entry.second));
            } else private_node.setParam(name, entry.second);
          }
          if (budget->failed()) throw xgc_scene::SceneError("ROS master did not retain scene launch parameters");
        }
        xgc_scene::SceneParameters config;
        {
          auto budget = binding.BoundMasterCalls(seconds(5));
          try {
            config = xgc_scene::SceneParameters::parse(parameters(private_node, options));
          } catch (const xgc_scene::SceneError& error) {
            stop(adapter::kExitUsage, error.what());
            return;
          }
          if (budget->failed()) throw xgc_scene::SceneError("ROS master did not supply scene parameters");
        }
        std::optional<sensor_msgs::PointCloud2> fixed_cloud;
        if (session) {
          const auto& facts = session->facts;
          const auto& runtime = facts.at("runtime");
          const auto selected = xgc_scene::inputs(facts, runtime.at("userFilesDir").get<std::string>(),
                                                 runtime.at("workspaceDir").get<std::string>());
          config.scene_file = selected.at("sceneFile").get<std::string>();
          config.working_file = selected.at("workingFile").get<std::string>();
          config.frozen = selected.at("frozen").get<bool>();
          config.generation_parameters = xgc_scene::parseJson(selected.at("generationParametersJson").get<std::string>());
          if (selected.contains("pointCloud")) {
            fixed_cloud = xgc_scene::fixedPointCloud(selected);
            lidar_parameters = xgc_scene::lidar_ros::Parameters::parse(lidar_values, profile, session, true);
            // The canonical reference topic has one owner. A fixed original
            // cloud is never replaced by the geometry fleet's surface sample.
            lidar_parameters.manifest.reference.reset();
            lidar_parameters.have_fleet = !lidar_parameters.manifest.robots.empty();
          }
        }
        std::shared_ptr<xgc_scene::SimulationClient> simulation;
        if (config.profile == "gazebo") simulation = std::make_shared<xgc_scene::SimulationClient>(config.simulation_endpoint, config.target_id);
        { std::lock_guard<std::mutex> lock(objects_mutex); objects.simulation = simulation; }
        if (stopping.load()) { if (simulation) simulation->close(); return; }
        std::shared_ptr<xgc_scene::RosPublishers> outputs;
        std::shared_ptr<xgc_scene::lidar_ros::Edge> lidar;
        {
          auto budget = binding.BoundMasterCalls(seconds(5));
          outputs = std::make_shared<xgc_scene::RosPublishers>(handle);
          lidar = std::make_shared<xgc_scene::lidar_ros::Edge>(handle, lidar_parameters);
          if (fixed_cloud) {
            outputs->fixedCloud(*fixed_cloud);
            if (lidar->engine()) lidar->engine()->setPointCloud(fixed_cloud->data, 0);
          }
          if (budget->failed()) throw xgc_scene::SceneError("ROS master did not register scene outputs");
        }
        auto tap = std::make_shared<xgc_scene::lidar_ros::SceneTap>(*outputs, lidar->engine(),
            [] { return ros::Time::now().toNSec(); });
        auto name_space = handle.getNamespace();
        // rospy.get_namespace() includes the trailing slash; NodeHandle does
        // not. Hash the Python spelling to preserve the public TF prefix.
        if (name_space.empty() || name_space.back() != '/') name_space += '/';
        auto node = std::make_shared<xgc_scene::SceneNode>(config, name_space, *tap,
            [] { return ros::Time::now().toSec(); }, simulation);
        {
          std::lock_guard<std::mutex> lock(objects_mutex);
          objects.outputs = outputs;
          objects.tap = tap;
          objects.lidar = lidar;
          objects.node = node;
        }
        if (!stopping.load()) {
          auto facts = lidar_parameters.facts(config.profile);
          facts["ros_master"] = "bound";
          lidar_control->SetFacts(facts, [lidar] { return lidar->peerFacts(); });
          lidar_control->SetReady(true);
          service->facts({{"run_id", options.run_id}, {"target_id", options.target_id}, {"profile", config.profile},
              {"ros_master", "bound"}, {"ros_master_uri", master.uri}, {"ros_master_pid", master.pid},
              {"ros_master_run_id", master.run_id}, {"ros_node", ros::this_node::getName()}}, [node] { return node->peerFacts(); });
          service->ready(std::make_shared<LaneAuthoring>(node, post, stopping), xgc_scene::Json{{"run_id", options.run_id}, {"target_id", options.target_id},
            {"profile", config.profile},
            {"ros_master_uri", master.uri}, {"ros_master_pid", master.pid}, {"ros_master_run_id", master.run_id},
            {"ros_node", ros::this_node::getName()}, {"ros_master", "bound"}});
        }
        auto timer = handle.createTimer(ros::Duration(1.0 / 30.0), [node](const ros::TimerEvent&) { node->tick(); });
        callbacks = std::thread([&] {
          try {
            while (!stopping.load() && binding.Running()) binding.PumpLane(0, milliseconds(100));
          } catch (const std::exception& error) { stop(adapter::kExitFailure, error.what()); }
        });
        std::optional<steady_clock::time_point> unreachable;
        while (!stopping.load() && binding.Running()) {
          adapter::MasterIdentity current;
          bool answered;
          { auto budget = binding.BoundMasterCalls(milliseconds(500)); answered = binding.ProbeMaster(&current); }
          if (answered) {
            unreachable.reset();
            if (!master.sameMaster(current)) { stop(adapter::kExitMasterChanged, "ROS master was replaced"); break; }
          } else {
            if (!unreachable) unreachable = steady_clock::now();
            if (steady_clock::now() - *unreachable >= seconds(5)) {
              stop(adapter::kExitMasterUnreachable, "ROS master has not answered for 5000 ms"); break;
            }
          }
          std::unique_lock<std::mutex> lock(stop_mutex);
          changed.wait_for(lock, seconds(1), [&] { return stopping.load(); });
        }
        if (!stopping.load()) stop(adapter::kExitRosShutdown, "ROS shut this node down");
        lidar_control->Stop();
        timer.stop();
        lidar->stopTimer();
      } catch (const std::exception& error) { if (!stopping.load()) stop(adapter::kExitFailure, error.what()); }
      stopping.store(true);
      changed.notify_all();
      lidar_control->Stop();
      std::shared_ptr<xgc_scene::SimulationClient> simulation;
      { std::lock_guard<std::mutex> lock(objects_mutex); simulation = objects.simulation; }
      if (simulation) simulation->close();
      binding.DisableLanes();
      if (callbacks.joinable()) callbacks.join();
    });

    while (!stopping.load() && !signalled) server.poll(milliseconds(100));
    stopping.store(true);
    changed.notify_all();
    lidar_control->Stop();
    service->stopping();
    std::shared_ptr<xgc_scene::SimulationClient> simulation;
    { std::lock_guard<std::mutex> lock(objects_mutex); simulation = objects.simulation; }
    if (simulation) simulation->close();
    if (lifecycle.joinable()) lifecycle.join();
    if (objects.node) objects.node->shutdown();
    service->close();
    server.drain_until(steady_clock::now() + seconds(2));
    if (ros_node) binding.FlushPublications(milliseconds(500));
    {
      auto budget = binding.BoundMasterCalls(seconds(2));
      objects.node.reset();
      objects.tap.reset();
      if (objects.lidar) objects.lidar->shutdown();
      objects.lidar.reset();
      objects.outputs.reset();
      ros_node.reset();
      if (ros_node || ros::isStarted()) binding.Shutdown();
      else ros::shutdown();
    }
    return exit_code.load();
  } catch (const std::exception& error) {
    stopping.store(true);
    changed.notify_all();
    if (lidar_control) lidar_control->Stop();
    std::shared_ptr<xgc_scene::SimulationClient> simulation;
    { std::lock_guard<std::mutex> lock(objects_mutex); simulation = objects.simulation; }
    if (simulation) simulation->close();
    if (lifecycle.joinable()) lifecycle.join();
    if (objects.node) objects.node->shutdown();
    if (service) service->close();
    objects.node.reset();
    objects.tap.reset();
    if (objects.lidar) objects.lidar->shutdown();
    objects.lidar.reset();
    objects.outputs.reset();
    ros_node.reset();
    if (ros::isInitialized()) { auto budget = binding.BoundMasterCalls(seconds(2)); binding.Shutdown(); }
    std::cerr << "scene adapter: " << error.what() << '\n';
    return adapter::kExitFailure;
  }
}
