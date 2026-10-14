#include "xgc_scene/lidar_ros/edge.hpp"
#include "xgc_scene/lidar_ros/scene_tap.hpp"
#include "xgc_scene/lidar/control.hpp"

#include <gtest/gtest.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/callback_queue.h>
#include <ros/master.h>
#include <xgc2/xrpc/http.hpp>
#include <filesystem>
#include <future>
#include <cstring>
#include <sstream>
#include <algorithm>
#include <unistd.h>

namespace xgc_scene::lidar_ros {
namespace {
using namespace std::chrono_literals;
struct Outputs : RosOutputs {
  void snapshot(const xgc2_geometry_msgs::SceneSnapshot&) override {}
  void state(const xgc2_geometry_msgs::SceneState&) override {}
  void document(const std::string&) override {}
  void markers(const visualization_msgs::MarkerArray&) override {}
  void staticTransforms(const std::vector<geometry_msgs::TransformStamped>&) override {}
  void dynamicTransforms(const std::vector<geometry_msgs::TransformStamped>&) override {}
};
class Dispatch : public ros::CallbackInterface {
 public:
  explicit Dispatch(std::function<void()> fn) : fn_(std::move(fn)) {}
  CallResult call() override { fn_(); return Success; }
 private:
  std::function<void()> fn_;
};

TEST(LidarTransport, OwnedMasterTwoWorkersDemandBodiesWireLatchAndControl) {
  if (!std::getenv("XGC_LIDAR_TEST_MASTER")) GTEST_SKIP() << "Run through test/ros/process_test.py --edge-test with an owned master";
  const auto* master = std::getenv("ROS_MASTER_URI");
  ASSERT_NE(master, nullptr);
  const auto uri = std::string(master);
  const auto port_at = uri.rfind(':');
  ASSERT_NE(port_at, std::string::npos);
  const auto port = std::stoi(uri.substr(port_at + 1));
  ASSERT_GE(port, 20000);
  ASSERT_LE(port, 40000);
  ASSERT_NE(std::getenv("ROS_HOME"), nullptr);
  ros::init(ros::M_string{}, "lidar_transport_" + std::to_string(::getpid()), ros::init_options::NoSigintHandler);
  struct Shutdown { ~Shutdown() { ros::shutdown(); } } shutdown;
  ros::CallbackQueue queue;
  ros::NodeHandle handle;
  handle.setCallbackQueue(&queue);
  const auto pump = [&](const std::function<bool()>& ready) {
    const auto until = std::chrono::steady_clock::now() + 5s;
    do {
      queue.callAvailable(ros::WallDuration(0.01));
      if (ready()) return true;
    } while (std::chrono::steady_clock::now() < until);
    return false;
  };
  Json robots = Json::array(), bodies = Json::array();
  const std::string prefix = "/lidar_edge_" + std::to_string(::getpid());
  for (int i = 0; i < 3; ++i) {
    const auto ns = prefix + "/body" + std::to_string(i);
    bodies.push_back({{"id", i}, {"namespace", ns}, {"poseTopic", ns + "/truth"},
        {"geometry", {{"type", "sphere"}, {"radiusMeters", 0.4}}}});
    if (i < 2) robots.push_back({{"namespace", ns}, {"mode", "raycast"}, {"hRes", 1}, {"vRes", 1},
        {"hFovDeg", 90}, {"vFovDeg", 0}, {"publishBeams", true}, {"rateHz", 10}});
  }
  auto config = Parameters::parse({{"lidar_xrpc_socket", "/tmp/unused-edge.sock"},
      {"lidar_defaults_json", "{\"worker_threads\":2}"}}, "gazebo");
  config.manifest = lidar::parseManifest(Json{{"schemaVersion", 1}, {"robots", robots}, {"bodies", bodies},
      {"referenceCloud", {{"surfaceSpacing", 0.5}}}}.dump());
  config.have_fleet = true;
  Edge edge(handle, config);
  edge.stopTimer();
  ASSERT_NE(edge.engine(), nullptr);
  ASSERT_EQ(edge.engine()->threads(), 2u);
  Outputs outputs;
  SceneTap tap(outputs, edge.engine(), [] { return ros::Time::now().toNSec(); });
  xgc2_geometry_msgs::SceneSnapshot definition;
  definition.header.frame_id = "world";
  definition.epoch = "first";
  definition.revision = 1;
  xgc2_geometry_msgs::SceneObstacle obstacle;
  obstacle.id = "wall";
  obstacle.pose.orientation.w = 1;
  obstacle.pose.position.x = 4;
  xgc2_geometry_msgs::ScenePart part;
  part.pose.orientation.w = 1;
  part.geometry.type = "box";
  part.geometry.size.x = 1;
  part.geometry.size.y = 4;
  part.geometry.size.z = 2;
  obstacle.parts.push_back(part);
  definition.obstacles.push_back(obstacle);
  tap.application("first", 1, true);
  tap.snapshot(definition);
  std::vector<ros::Publisher> poses;
  for (int i = 0; i < 3; ++i) poses.push_back(handle.advertise<geometry_msgs::PoseStamped>(bodies[i]["poseTopic"].get<std::string>(), 1));
  std::vector<ros::Subscriber> pose_observers;
  std::vector<std::uint64_t> pose_stamps(3);
  for (int i = 0; i < 3; ++i)
    pose_observers.push_back(handle.subscribe<geometry_msgs::PoseStamped>(bodies[i]["poseTopic"].get<std::string>(), 1,
        [&, i](const geometry_msgs::PoseStamped::ConstPtr& pose) { pose_stamps[i] = pose->header.stamp.toNSec(); }));
  ASSERT_TRUE(pump([&] { return poses[0].getNumSubscribers() && poses[1].getNumSubscribers() && poses[2].getNumSubscribers(); }));
  const auto send = [&](std::int64_t now, bool invalid = false) {
    for (int i = 0; i < 3; ++i) {
      geometry_msgs::PoseStamped pose;
      pose.header.frame_id = invalid && i == 2 ? "map" : "world";
      pose.header.stamp.fromNSec(now - 12345);
      pose.pose.orientation.w = 1;
      pose.pose.position.x = i == 2 ? 3 : 0;
      pose.pose.position.y = i == 1 ? 1 : 0;
      poses[i].publish(pose);
    }
    // Serialization finishes in publish; transport delivery is asynchronous.
    EXPECT_TRUE(pump([&] {
      return std::all_of(pose_stamps.begin(), pose_stamps.end(),
          [&](auto stamp) { return stamp == static_cast<std::uint64_t>(now - 12345); });
    }));
  };
  auto now = static_cast<std::int64_t>(ros::Time::now().toNSec());
  send(now);
  EXPECT_EQ(edge.engine()->tick(now), 0u);  // No subscribers means no work.
  std::vector<sensor_msgs::PointCloud2::ConstPtr> points(2), beams(2);
  std::vector<unsigned> received(2), beam_received(2);
  std::vector<ros::Subscriber> subscriptions;
  for (int i = 0; i < 2; ++i) {
    const auto ns = robots[i]["namespace"].get<std::string>();
    subscriptions.push_back(handle.subscribe<sensor_msgs::PointCloud2>(ns + "/simple_lidar/points", 1,
        [&, i](const sensor_msgs::PointCloud2::ConstPtr& cloud) { points[i] = cloud; ++received[i]; }));
    subscriptions.push_back(handle.subscribe<sensor_msgs::PointCloud2>(ns + "/simple_lidar/beams", 1,
        [&, i](const sensor_msgs::PointCloud2::ConstPtr& cloud) { beams[i] = cloud; ++beam_received[i]; }));
  }
  sensor_msgs::PointCloud2::ConstPtr reference;
  auto reference_sub = handle.subscribe<sensor_msgs::PointCloud2>("/xgc/scene/reference_cloud", 1,
      [&](const sensor_msgs::PointCloud2::ConstPtr& cloud) { reference = cloud; });
  ASSERT_TRUE(pump([&] {
    edge.engine()->tick(now);
    return points[0] && points[1] && beams[0] && beams[1] && reference && reference->width;
  }));
  ros::master::V_TopicInfo topics;
  ASSERT_TRUE(ros::master::getTopics(topics));
  EXPECT_FALSE(std::any_of(topics.begin(), topics.end(), [&](const auto& topic) {
    return topic.name.find(prefix + "/body2/simple_lidar/") == 0;
  }));
  EXPECT_EQ(points[0]->header.frame_id, "world");
  EXPECT_EQ(points[0]->header.stamp.toNSec(), now - 12345);
  EXPECT_EQ(points[0]->point_step, 12u);
  EXPECT_EQ(points[0]->fields.size(), 3u);
  EXPECT_EQ(points[0]->width, 1u);  // Unsensed body still returns a hit.
  EXPECT_EQ(points[0]->height, 1u);
  EXPECT_TRUE(points[0]->is_dense);
  float hit_x;
  std::memcpy(&hit_x, points[0]->data.data(), sizeof hit_x);
  EXPECT_FLOAT_EQ(hit_x, 2.6f);
  EXPECT_EQ(beams[0]->point_step, 32u);
  EXPECT_EQ(beams[0]->fields.back().name, "hit");
  EXPECT_EQ(beams[0]->header.stamp, points[0]->header.stamp);
  const auto first = points[0];
  const auto first_stamp = first->header.stamp;
  for (int i = 1; i < 21; ++i) {
    const auto stamp = now + i * 100000000;
    send(stamp);
    EXPECT_EQ(edge.engine()->tick(stamp), 2u);
    ASSERT_TRUE(pump([&] { return points[0]->header.stamp.toNSec() == static_cast<std::uint64_t>(stamp - 12345); }));
  }
  EXPECT_EQ(first->header.stamp, first_stamp);  // Reused source cannot mutate deliveries.
  now += 2100000000;
  send(now, true);
  EXPECT_EQ(edge.engine()->tick(now), 0u);  // Invalid unsensed truth fails closed.
  now += 100000000;
  send(now);
  EXPECT_EQ(edge.engine()->tick(now), 2u);

  char pattern[] = "/tmp/lf-edge-control-XXXXXX";
  const auto* directory = ::mkdtemp(pattern);
  ASSERT_NE(directory, nullptr);
  struct Directory { std::string path; ~Directory() { std::filesystem::remove_all(path); } } cleanup{directory};
  const auto socket = cleanup.path + "/control.sock";
  lidar::Control control(socket, "fixture", true,
      [&](std::function<void()> fn) { queue.addCallback(ros::CallbackInterfacePtr(new Dispatch(std::move(fn)))); },
      [&](bool enabled) { edge.setEnabled(enabled); }, config.facts("gazebo"));
  control.Start();
  const auto call = [&](std::string method, std::string route, std::string instance, Json body = Json()) {
    xgc2::xrpc::HttpClient client(socket, {}, instance);
    xgc2::xrpc::HttpRequest request;
    request.method = std::move(method);
    request.target = std::move(route);
    if (!body.is_null()) { request.body = body.dump(); request.request_id = xgc2::xrpc::new_instance_id(); }
    return Json::parse(client.call(request, xgc2::xrpc::Clock::now() + 3s).body);
  };
  const auto describe = call("GET", "/v1/describe?wait_ready_ms=0", "");
  const auto instance = describe.at("instance_id").get<std::string>();
  const auto configure = [&](bool enabled, int revision) {
    auto result = std::async(std::launch::async, [&] {
      return call("POST", "/v1/configure", instance, Json{{"expected_revision", revision},
          {"configuration", {{"enabled", enabled}}}, {"operation_timeout_ms", 2000}});
    });
    EXPECT_TRUE(pump([&] { return result.wait_for(0ms) == std::future_status::ready; }));
    EXPECT_EQ(result.get().at("state"), "succeeded");
  };
  configure(false, 1);
  now += 100000000;
  send(now);
  EXPECT_EQ(edge.engine()->tick(now), 0u);
  configure(true, 2);
  now += 100000000;
  send(now);
  EXPECT_EQ(edge.engine()->tick(now), 2u);

  sensor_msgs::PointCloud2::ConstPtr late_reference;
  auto late_sub = handle.subscribe<sensor_msgs::PointCloud2>("/xgc/scene/reference_cloud", 1,
      [&](const sensor_msgs::PointCloud2::ConstPtr& cloud) { late_reference = cloud; });
  ASSERT_TRUE(pump([&] { return late_reference && late_reference->width; }));
  tap.application("second", 1, false);
  ASSERT_TRUE(pump([&] { return late_reference->width == 0; }));
  EXPECT_EQ(late_reference->point_step, 12u);
  definition.epoch = "second";
  definition.obstacles[0].dynamic = true;
  tap.snapshot(definition);
  tap.application("second", 1, true);
  EXPECT_FALSE(edge.engine()->sceneReady());
  late_sub.shutdown();
  late_reference.reset();
  late_sub = handle.subscribe<sensor_msgs::PointCloud2>("/xgc/scene/reference_cloud", 1,
      [&](const sensor_msgs::PointCloud2::ConstPtr& cloud) { late_reference = cloud; });
  ASSERT_TRUE(pump([&] { return late_reference != nullptr; }));
  EXPECT_EQ(late_reference->width, 0u);  // The empty replacement itself is latched.
  xgc2_geometry_msgs::SceneState current;
  current.header.frame_id = "world";
  current.epoch = "second";
  current.revision = 1;
  xgc2_geometry_msgs::SceneObstacleState body;
  body.id = "wall";
  body.pose = obstacle.pose;
  current.obstacles.push_back(body);
  tap.state(current);
  now += 100000000;
  send(now);
  EXPECT_EQ(edge.engine()->tick(now), 2u);
  ASSERT_TRUE(pump([&] { return late_reference->width > 0; }));
  subscriptions[0].shutdown();
  subscriptions[2].shutdown();
  queue.callAvailable(ros::WallDuration(0.02));
  now += 100000000;
  send(now);
  EXPECT_EQ(edge.engine()->tick(now), 2u);  // Beam subscribers alone demand scans.
  for (auto& subscriber : subscriptions) subscriber.shutdown();
  queue.callAvailable(ros::WallDuration(0.02));
  now += 100000000;
  send(now);
  EXPECT_EQ(edge.engine()->tick(now), 0u);
  control.Stop();

  // The production timer follows ROS simulated time and cannot advance with
  // wall time while the simulation clock is paused.
  ros::Time::setNow(ros::Time(100));
  EXPECT_TRUE(ros::Time::isSimTime());
  const auto clock_ns = prefix + "/clock_sensor";
  const auto clock_session = lidar::parseSessionInput(Json{{"runMode", "simulation"},
      {"robots", Json::array({{{"namespace", clock_ns}, {"binding", {{"simulationSensors", {{"simpleLidar",
          {{"enabled", true}, {"mode", "depth_frustum"}, {"acceleration", "cpu"},
           {"hRes", 1}, {"vRes", 1}, {"rateHz", 10}}}}}}}}})}, {"visualizationTopics", Json::array()}}.dump());
  const auto clock_config = Parameters::parse({{"lidar_xrpc_socket", "/tmp/unused-clock.sock"}}, "gazebo", clock_session);
  Edge clock_edge(handle, clock_config);
  clock_edge.engine()->setScene({xgc2_math::scene::Obstacle::box({4, 0, 0}, {1, 4, 2})}, 1);
  unsigned clock_deliveries = 0;
  auto clock_sub = handle.subscribe<sensor_msgs::PointCloud2>(clock_ns + "/simple_lidar/points", 1,
      [&](const sensor_msgs::PointCloud2::ConstPtr& cloud) {
        ++clock_deliveries;
        EXPECT_EQ(cloud->header.stamp, ros::Time(100));
      });
  auto clock_pose = handle.advertise<geometry_msgs::PoseStamped>(clock_ns + "/pose", 1);
  ASSERT_TRUE(pump([&] { return clock_pose.getNumSubscribers() > 0; }));
  geometry_msgs::PoseStamped clock_sample;
  clock_sample.header.frame_id = "world";
  clock_sample.header.stamp = ros::Time(100);
  clock_sample.pose.orientation.w = 1;
  clock_pose.publish(clock_sample);
  queue.callAvailable(ros::WallDuration(0.1));
  EXPECT_EQ(clock_deliveries, 0u);
  ros::Time::setNow(ros::Time(100, 100000000));
  ASSERT_TRUE(pump([&] { return clock_deliveries == 1; }));
  const auto paused_until = std::chrono::steady_clock::now() + 200ms;
  while (std::chrono::steady_clock::now() < paused_until) queue.callAvailable(ros::WallDuration(0.01));
  EXPECT_EQ(clock_deliveries, 1u);
  ros::Time::setNow(ros::Time(100, 200000000));
  ASSERT_TRUE(pump([&] { return clock_deliveries == 2; }));
  clock_edge.stopTimer();
  clock_edge.shutdown();
  edge.shutdown();
  EXPECT_EQ(edge.engine(), nullptr);
  reference_sub.shutdown();
  late_sub.shutdown();
  reference.reset();
  const auto reference_session = lidar::parseSessionInput(Json{{"runMode", "simulation"}, {"robots", Json::array()},
      {"visualizationTopics", Json::array({{{"topic", "/xgc/scene/reference_cloud"},
          {"messageType", "sensor_msgs/PointCloud2"}, {"role", "semantic"}, {"visible3d", true}}})}}.dump());
  const auto reference_config = Parameters::parse({{"lidar_xrpc_socket", "/tmp/unused-reference.sock"}}, "gazebo", reference_session);
  Edge reference_edge(handle, reference_config);
  reference_edge.stopTimer();
  ASSERT_TRUE(reference_edge.engine()->manifest().robots.empty());
  reference_edge.engine()->setScene({xgc2_math::scene::Obstacle::box({4, 0, 0}, {1, 4, 2})}, 1);
  reference_sub = handle.subscribe<sensor_msgs::PointCloud2>("/xgc/scene/reference_cloud", 1,
      [&](const sensor_msgs::PointCloud2::ConstPtr& cloud) { reference = cloud; });
  ASSERT_TRUE(pump([&] {
    reference_edge.engine()->tick(ros::Time::now().toNSec());
    return reference && reference->width;
  }));
  ASSERT_TRUE(ros::master::getTopics(topics));
  EXPECT_FALSE(std::any_of(topics.begin(), topics.end(), [&](const auto& topic) {
    return topic.name.find(prefix + "/") == 0 && topic.name.find("/simple_lidar/") != std::string::npos;
  }));
  EXPECT_EQ(reference->point_step, 12u);
  EXPECT_EQ(reference->fields.size(), 3u);
  reference_edge.shutdown();

  const auto sampled_ns = prefix + "/sampled";
  const auto native_ns = prefix + "/native";
  const auto roster = Json{{"runMode", "simulation"}, {"robots", Json::array({
      {{"namespace", sampled_ns}, {"binding", {{"simulationSensors", {{"simpleLidar", {{"enabled", true},
          {"mode", "depth_frustum"}, {"acceleration", "cpu"}, {"hRes", 1}, {"vRes", 1}}}}}}}},
      {{"namespace", native_ns}, {"binding", {{"simulationSensors", {{"simpleLidar", true}}}}}}
      })}, {"visualizationTopics", Json::array()}};
  std::istringstream session_stream(roster.dump());
  const auto roster_config = Parameters::parse({{"lidar_xrpc_socket", "/tmp/unused-roster.sock"}},
      "gazebo", readSessionInput(session_stream));
  ASSERT_EQ(roster_config.manifest.robots.size(), 1u);
  EXPECT_EQ(roster_config.manifest.robots[0].pose_topic, sampled_ns + "/pose");
  Edge roster_edge(handle, roster_config);
  roster_edge.stopTimer();
  roster_edge.engine()->setScene({xgc2_math::scene::Obstacle::box({4, 0, 0}, {1, 4, 2})}, 1);
  sensor_msgs::PointCloud2::ConstPtr roster_cloud;
  auto roster_sub = handle.subscribe<sensor_msgs::PointCloud2>(sampled_ns + "/simple_lidar/points", 1,
      [&](const sensor_msgs::PointCloud2::ConstPtr& cloud) { roster_cloud = cloud; });
  auto roster_pose = handle.advertise<geometry_msgs::PoseStamped>(sampled_ns + "/pose", 1);
  auto retired_roster_pose = handle.advertise<geometry_msgs::PoseStamped>(
      "/vrpn_client_node" + sampled_ns + "/pose", 1);
  ASSERT_TRUE(pump([&] { return roster_pose.getNumSubscribers() > 0; }));
  EXPECT_EQ(retired_roster_pose.getNumSubscribers(), 0u);
  clock_sample.header.stamp = ros::Time::now();
  roster_pose.publish(clock_sample);
  ASSERT_TRUE(pump([&] {
    roster_edge.engine()->tick(ros::Time::now().toNSec());
    return roster_cloud != nullptr;
  }));
  EXPECT_EQ(roster_cloud->header.stamp, clock_sample.header.stamp);
  EXPECT_GT(roster_cloud->width, 0u);
  ASSERT_TRUE(ros::master::getTopics(topics));
  EXPECT_FALSE(std::any_of(topics.begin(), topics.end(), [&](const auto& topic) {
    return topic.name == sampled_ns + "/simple_lidar/beams" || topic.name.find(native_ns + "/simple_lidar/") == 0;
  }));
  roster_edge.shutdown();
}
}  // namespace
}  // namespace xgc_scene::lidar_ros
