#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3.h>
#include <grpcpp/grpcpp.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

#include "adapter/v1/adapter.grpc.pb.h"
#include "xgc_ros1_adapter/robot_behavior.hpp"

using xgc_ros1_adapter::RobotRuntime;
using xgc_ros1_adapter::RobotFeature;
using xgc_ros1_adapter::behaviorForType;
using xgc_ros1_adapter::fieldBool;
using xgc_ros1_adapter::fieldDouble;
using xgc_ros1_adapter::fieldString;
using xgc_ros1_adapter::hasFeature;
using xgc_ros1_adapter::topic;
using xgc_ros1_adapter::trimSlash;

class Ros1AdapterNode {
 public:
  Ros1AdapterNode(ros::NodeHandle nh, ros::NodeHandle private_nh)
      : nh_(std::move(nh)), private_nh_(std::move(private_nh)) {
    private_nh_.param<std::string>("adapter_id", adapter_id_, "ros1-adapter");
    private_nh_.param<std::string>("experiment_id", experiment_id_, "exp-ros1");
    private_nh_.param<std::string>("socket_path", socket_path_, "/tmp/xgc2/adapter-ingress.sock");
    private_nh_.param<double>("publish_rate_hz", publish_rate_hz_, 10.0);
    private_nh_.param<double>("remote_publish_rate_hz", remote_publish_rate_hz_, 20.0);

    auto channel = grpc::CreateChannel("unix:" + socket_path_, grpc::InsecureChannelCredentials());
    stub_ = xgc::adapter::v1::AdapterIngress::NewStub(channel);

    loadAdapterConfig();
    installRosInterfaces();
    registerAdapter();

    const double state_period = publish_rate_hz_ > 0.0 ? 1.0 / publish_rate_hz_ : 0.1;
    const double remote_period = remote_publish_rate_hz_ > 0.0 ? 1.0 / remote_publish_rate_hz_ : 0.05;
    publish_timer_ = nh_.createTimer(ros::Duration(state_period), &Ros1AdapterNode::publishState, this);
    heartbeat_timer_ = nh_.createTimer(ros::Duration(1.0), &Ros1AdapterNode::heartbeat, this);
    metrics_timer_ = nh_.createTimer(ros::Duration(1.0), &Ros1AdapterNode::updateTopicMetrics, this);
    remote_timer_ = nh_.createTimer(ros::Duration(remote_period), &Ros1AdapterNode::remoteControlTimer, this);
    command_thread_ = std::thread(&Ros1AdapterNode::commandStreamLoop, this);
  }

  ~Ros1AdapterNode() {
    std::shared_ptr<grpc::ClientContext> context;
    {
      std::lock_guard<std::mutex> lock(mu_);
      shutting_down_ = true;
      context = active_command_context_;
    }
    if (context) {
      context->TryCancel();
    }
    publishStopAll();
    if (command_thread_.joinable()) {
      command_thread_.join();
    }
  }

 private:
  void loadAdapterConfig() {
    xgc::adapter::v1::GetAdapterConfigRequest request;
    request.set_adapter_id(adapter_id_);
    xgc::adapter::v1::GetAdapterConfigResponse response;
    grpc::ClientContext context;
    const auto status = stub_->GetAdapterConfig(&context, request, &response);
    if (status.ok() && response.accepted() && response.robots_size() > 0) {
      for (const auto& robot : response.robots()) {
        addRobotConfig(robot);
      }
      ROS_INFO_STREAM("XGC ROS1 adapter loaded " << robots_.size() << " robot mappings");
      return;
    }
    ROS_WARN_STREAM("XGC adapter config unavailable, falling back to ROS params: " << status.error_message());
    RobotRuntime robot;
    private_nh_.param<std::string>("robot_id", robot.robot_id, "ugv1");
    private_nh_.param<std::string>("robot_type", robot.robot_type, "ugv");
    private_nh_.param<std::string>("robot_namespace", robot.robot_namespace, "ugv1");
    behaviorForType(robot.robot_type).set_default_endpoints(robot);
    private_nh_.param<std::string>("pose_topic", robot.pose_topic, topic(robot.robot_namespace, "pose"));
    private_nh_.param<std::string>("twist_topic", robot.twist_topic, topic(robot.robot_namespace, "twist"));
    private_nh_.param<std::string>("imu_topic", robot.imu_topic, topic(robot.robot_namespace, "imu"));
    private_nh_.param<std::string>("cmd_vel_topic", robot.cmd_vel_topic, topic(robot.robot_namespace, "cmd_vel"));
    private_nh_.param<std::string>("takeoff_topic", robot.takeoff_topic, topic(robot.robot_namespace, "takeoff"));
    private_nh_.param<std::string>("land_topic", robot.land_topic, topic(robot.robot_namespace, "land"));
    private_nh_.param<std::string>("state_topic", robot.state_topic, topic(robot.robot_namespace, "mavros/state"));
    private_nh_.param<std::string>("extended_state_topic", robot.extended_state_topic, topic(robot.robot_namespace, "mavros/extended_state"));
    private_nh_.param<std::string>("battery_topic", robot.battery_topic, topic(robot.robot_namespace, "mavros/battery"));
    private_nh_.param<std::string>("global_position_topic", robot.global_position_topic, topic(robot.robot_namespace, "mavros/global_position/global"));
    private_nh_.param<std::string>("stage_topic", robot.stage_topic, topic(robot.robot_namespace, "stage"));
    private_nh_.param<std::string>("angle_topic", robot.angle_topic, topic(robot.robot_namespace, "angle"));
    private_nh_.param<std::string>("ias_topic", robot.ias_topic, topic(robot.robot_namespace, "ias"));
    private_nh_.param<std::string>("pos_topic", robot.pos_topic, topic(robot.robot_namespace, "pos"));
    private_nh_.param<std::string>("baro_topic", robot.baro_topic, topic(robot.robot_namespace, "baro"));
    private_nh_.param<std::string>("arm_service", robot.arm_service, topic(robot.robot_namespace, "mavros/cmd/arming"));
    private_nh_.param<std::string>("mode_service", robot.mode_service, topic(robot.robot_namespace, "mavros/set_mode"));
    private_nh_.param<std::string>("command_service", robot.command_service, topic(robot.robot_namespace, "mavros/cmd/command"));
    private_nh_.param<std::string>("takeoff_service", robot.takeoff_service, topic(robot.robot_namespace, "mavros/cmd/takeoff"));
    private_nh_.param<std::string>("land_service", robot.land_service, topic(robot.robot_namespace, "mavros/cmd/land"));
    robot.remote_publish_rate_hz = remote_publish_rate_hz_;
    robots_[robot.robot_id] = robot;
  }

  void addRobotConfig(const xgc::adapter::v1::RobotMapping& mapping) {
    RobotRuntime robot;
    robot.robot_id = mapping.robot_id().empty() ? "ugv1" : mapping.robot_id();
    robot.robot_type = mapping.robot_type().empty() ? "ugv" : mapping.robot_type();
    robot.robot_namespace = mapping.namespace_().empty() ? robot.robot_id : mapping.namespace_();
    behaviorForType(robot.robot_type).set_default_endpoints(robot);
    behaviorForType(robot.robot_type).apply_mapping(mapping, robot);
    if (mapping.has_remote_control()) {
      const auto& remote = mapping.remote_control();
      if (!remote.command_topic().empty()) {
        robot.cmd_vel_topic = remote.command_topic();
      }
      if (remote.publish_rate_hz() > 0.0) {
        robot.remote_publish_rate_hz = remote.publish_rate_hz();
      } else {
        robot.remote_publish_rate_hz = remote_publish_rate_hz_;
      }
    }
    robots_[robot.robot_id] = robot;
  }

  void installRosInterfaces() {
    for (auto& entry : robots_) {
      const std::string robot_id = entry.first;
      auto& robot = entry.second;
      const auto& behavior = behaviorForType(robot.robot_type);
      const bool topic_signal = hasFeature(behavior, RobotFeature::TopicSignal);
      const bool mavros_services = hasFeature(behavior, RobotFeature::MavrosServices);
      const bool mavros_telemetry = hasFeature(behavior, RobotFeature::MavrosTelemetry);
      const bool fixed_wing_telemetry = hasFeature(behavior, RobotFeature::FixedWingTelemetry);
      const bool px4_rotor = robot.robot_type == "px4" || robot.robot_type == "minidrone";
      if (px4_rotor) {
        robot.position_target_pub = nh_.advertise<mavros_msgs::PositionTarget>(robot.cmd_vel_topic, 5);
      } else {
        robot.cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>(robot.cmd_vel_topic, 5);
      }
      if (topic_signal) {
        robot.takeoff_pub = nh_.advertise<std_msgs::Empty>(robot.takeoff_topic, 2);
        robot.land_pub = nh_.advertise<std_msgs::Empty>(robot.land_topic, 2);
      }
      if (mavros_services) {
        robot.arm_client = nh_.serviceClient<mavros_msgs::CommandBool>(robot.arm_service);
        robot.mode_client = nh_.serviceClient<mavros_msgs::SetMode>(robot.mode_service);
        robot.command_client = nh_.serviceClient<mavros_msgs::CommandLong>(robot.command_service);
        robot.takeoff_client = nh_.serviceClient<mavros_msgs::CommandTOL>(robot.takeoff_service);
        robot.land_client = nh_.serviceClient<mavros_msgs::CommandTOL>(robot.land_service);
      }
      robot.pose_sub = nh_.subscribe<geometry_msgs::PoseStamped>(
          robot.pose_topic, 10, [this, robot_id](const geometry_msgs::PoseStamped::ConstPtr& msg) {
            poseCallback(robot_id, msg);
          });
      if (hasFeature(behavior, RobotFeature::PlainTwist)) {
        robot.twist_plain_sub = nh_.subscribe<geometry_msgs::Twist>(
            robot.twist_topic, 10, [this, robot_id](const geometry_msgs::Twist::ConstPtr& msg) {
              twistPlainCallback(robot_id, msg);
            });
      } else {
        robot.twist_sub = nh_.subscribe<geometry_msgs::TwistStamped>(
            robot.twist_topic, 10, [this, robot_id](const geometry_msgs::TwistStamped::ConstPtr& msg) {
              twistCallback(robot_id, msg);
            });
      }
      robot.imu_sub = nh_.subscribe<sensor_msgs::Imu>(
          robot.imu_topic, 10, [this, robot_id](const sensor_msgs::Imu::ConstPtr& msg) {
            imuCallback(robot_id, msg);
          });
      if (px4_rotor) {
        robot.position_target_sub = nh_.subscribe<mavros_msgs::PositionTarget>(
            robot.cmd_vel_topic, 10, [this, robot_id](const mavros_msgs::PositionTarget::ConstPtr& msg) {
              positionTargetCallback(robot_id, msg);
            });
      } else {
        robot.cmd_vel_sub = nh_.subscribe<geometry_msgs::Twist>(
            robot.cmd_vel_topic, 10, [this, robot_id](const geometry_msgs::Twist::ConstPtr& msg) {
              cmdVelCallback(robot_id, msg);
            });
      }
      if (mavros_telemetry || fixed_wing_telemetry) {
        robot.state_sub = nh_.subscribe<mavros_msgs::State>(
            robot.state_topic, 10, [this, robot_id](const mavros_msgs::State::ConstPtr& msg) {
              mavrosStateCallback(robot_id, msg);
            });
      }
      if (mavros_telemetry) {
        robot.extended_state_sub = nh_.subscribe<mavros_msgs::ExtendedState>(
            robot.extended_state_topic, 10, [this, robot_id](const mavros_msgs::ExtendedState::ConstPtr& msg) {
              extendedStateCallback(robot_id, msg);
            });
        robot.battery_sub = nh_.subscribe<sensor_msgs::BatteryState>(
            robot.battery_topic, 10, [this, robot_id](const sensor_msgs::BatteryState::ConstPtr& msg) {
              batteryCallback(robot_id, msg);
            });
      }
      if (fixed_wing_telemetry) {
        robot.global_position_sub = nh_.subscribe<sensor_msgs::NavSatFix>(
            robot.global_position_topic, 10, [this, robot_id](const sensor_msgs::NavSatFix::ConstPtr& msg) {
              globalPositionCallback(robot_id, msg);
            });
        robot.stage_sub = nh_.subscribe<std_msgs::Int32>(
            robot.stage_topic, 10, [this, robot_id](const std_msgs::Int32::ConstPtr& msg) {
              stageCallback(robot_id, msg);
            });
        robot.angle_sub = nh_.subscribe<geometry_msgs::Vector3>(
            robot.angle_topic, 10, [this, robot_id](const geometry_msgs::Vector3::ConstPtr& msg) {
              angleCallback(robot_id, msg);
            });
        robot.ias_sub = nh_.subscribe<std_msgs::Float32>(
            robot.ias_topic, 10, [this, robot_id](const std_msgs::Float32::ConstPtr& msg) {
              iasCallback(robot_id, msg);
            });
        robot.pos_sub = nh_.subscribe<geometry_msgs::Vector3>(
            robot.pos_topic, 10, [this, robot_id](const geometry_msgs::Vector3::ConstPtr& msg) {
              posCallback(robot_id, msg);
            });
        robot.baro_sub = nh_.subscribe<std_msgs::Float32>(
            robot.baro_topic, 10, [this, robot_id](const std_msgs::Float32::ConstPtr& msg) {
              baroCallback(robot_id, msg);
            });
      }
      ROS_INFO_STREAM("XGC ROS1 adapter robot=" << robot_id << " pose=" << robot.pose_topic
                                                << " imu=" << robot.imu_topic
                                                << " cmd_vel=" << robot.cmd_vel_topic
                                                << " takeoff=" << robot.takeoff_topic
                                                << " land=" << robot.land_topic
                                                << " state=" << robot.state_topic);
    }
  }

  void registerAdapter() {
    xgc::adapter::v1::RegisterAdapterRequest request;
    request.set_adapter_id(adapter_id_);
    request.set_protocol("ros1");
    request.set_version("0.2.0");
    for (const auto& entry : robots_) {
      const auto& robot = entry.second;
      auto* out = request.add_robots();
      behaviorForType(robot.robot_type).append_registration(robot, out);
    }
    xgc::adapter::v1::RegisterAdapterResponse response;
    grpc::ClientContext context;
    const auto status = stub_->RegisterAdapter(&context, request, &response);
    if (!status.ok() || !response.accepted()) {
      ROS_WARN_STREAM("XGC adapter registration failed: " << status.error_message());
      return;
    }
    ROS_INFO_STREAM("XGC adapter registered with core " << response.core_id());
  }

  void heartbeat(const ros::TimerEvent&) {
    xgc::adapter::v1::HeartbeatRequest request;
    request.set_adapter_id(adapter_id_);
    request.set_observed_unix_nanos(ros::WallTime::now().toNSec());
    xgc::adapter::v1::HeartbeatResponse response;
    grpc::ClientContext context;
    const auto status = stub_->Heartbeat(&context, request, &response);
    if (!status.ok()) {
      ROS_WARN_THROTTLE(5.0, "XGC adapter heartbeat failed: %s", status.error_message().c_str());
    }
  }

  void commandStreamLoop() {
    while (ros::ok() && !isShuttingDown()) {
      xgc::adapter::v1::CommandStreamRequest request;
      request.set_adapter_id(adapter_id_);
      {
        std::lock_guard<std::mutex> lock(mu_);
        for (const auto& entry : robots_) {
          request.add_robot_ids(entry.first);
        }
      }
      auto context = std::make_shared<grpc::ClientContext>();
      {
        std::lock_guard<std::mutex> lock(mu_);
        active_command_context_ = context;
      }
      auto reader = stub_->StreamCommands(context.get(), request);
      xgc::adapter::v1::AdapterCommand command;
      while (ros::ok() && !isShuttingDown() && reader->Read(&command)) {
        handleCommand(command);
      }
      const auto status = reader->Finish();
      {
        std::lock_guard<std::mutex> lock(mu_);
        if (active_command_context_ == context) {
          active_command_context_.reset();
        }
      }
      if (ros::ok() && !isShuttingDown()) {
        ROS_WARN_THROTTLE(5.0, "XGC adapter command stream closed: %s", status.error_message().c_str());
        ros::Duration(1.0).sleep();
      }
    }
  }

  void handleCommand(const xgc::adapter::v1::AdapterCommand& command) {
    if (command.has_service_call()) {
      callServiceCommand(command);
      return;
    }
    if (command.has_topic_signal()) {
      publishTopicSignal(command);
      return;
    }
    if (!command.has_remote_control()) {
      return;
    }
    const auto& remote = command.remote_control();
    geometry_msgs::Twist immediate_stop;
    bool should_publish_stop = false;
    ros::Publisher publisher;
    bool px4_rotor = false;
    {
      std::lock_guard<std::mutex> lock(mu_);
      auto it = robots_.find(command.robot_id());
      if (it == robots_.end()) {
        return;
      }
      auto& robot = it->second;
      px4_rotor = robot.robot_type == "px4" || robot.robot_type == "minidrone";
      publisher = px4_rotor ? robot.position_target_pub : robot.cmd_vel_pub;
      if (remote.stop() || !remote.enabled()) {
        robot.remote_enabled = false;
        robot.current_remote_cmd = geometry_msgs::Twist();
        robot.current_position_target_cmd = mavros_msgs::PositionTarget();
        robot.pending_stop_publish = true;
        should_publish_stop = !px4_rotor;
      } else {
        robot.current_remote_cmd = geometry_msgs::Twist();
        robot.current_remote_cmd.linear.x = remote.linear_x();
        robot.current_remote_cmd.linear.y = remote.linear_y();
        robot.current_remote_cmd.linear.z = remote.linear_z();
        robot.current_remote_cmd.angular.x = remote.angular_x();
        robot.current_remote_cmd.angular.y = remote.angular_y();
        robot.current_remote_cmd.angular.z = remote.angular_z();
        if (px4_rotor) {
          robot.current_position_target_cmd = px4PositionTarget(remote);
        }
        robot.remote_enabled = true;
        robot.pending_stop_publish = false;
      }
    }
    if (should_publish_stop && publisher) {
      publisher.publish(immediate_stop);
    }
  }

  void callServiceCommand(const xgc::adapter::v1::AdapterCommand& command) {
    RobotRuntime robot;
    {
      std::lock_guard<std::mutex> lock(mu_);
      auto it = robots_.find(command.robot_id());
      if (it == robots_.end()) {
        return;
      }
      robot = it->second;
    }
    const auto& service = command.service_call();
    std::string service_type = service.service_type();
    if (service_type.empty()) {
      if (command.capability() == "robot.arm" || command.capability() == "robot.disarm") {
        service_type = "mavros_msgs/CommandBool";
      } else if (command.capability() == "robot.mode" || command.capability() == "robot.return" ||
                 command.capability() == "robot.hold") {
        service_type = "mavros_msgs/SetMode";
      } else if (command.capability() == "robot.takeoff" || command.capability() == "robot.land") {
        service_type = "mavros_msgs/CommandTOL";
      }
    }
    if (service_type == "mavros_msgs/CommandBool") {
      mavros_msgs::CommandBool srv;
      srv.request.value = fieldBool(service.fields(), "value", command.capability() == "robot.arm");
      if (!robot.arm_client.call(srv)) {
        ROS_WARN_STREAM("XGC ROS1 adapter service failed: " << robot.arm_service);
      }
      return;
    }
    if (service_type == "mavros_msgs/SetMode") {
      mavros_msgs::SetMode srv;
      srv.request.base_mode = static_cast<uint8_t>(fieldDouble(service.fields(), "base_mode", 0));
      srv.request.custom_mode = fieldString(service.fields(), "custom_mode", "OFFBOARD");
      if (!robot.mode_client.call(srv)) {
        ROS_WARN_STREAM("XGC ROS1 adapter service failed: " << robot.mode_service);
      }
      return;
    }
    if (service_type == "mavros_msgs/CommandTOL") {
      mavros_msgs::CommandTOL srv;
      srv.request.min_pitch = fieldDouble(service.fields(), "min_pitch", 0.0);
      srv.request.yaw = fieldDouble(service.fields(), "yaw", 0.0);
      srv.request.latitude = fieldDouble(service.fields(), "latitude", 0.0);
      srv.request.longitude = fieldDouble(service.fields(), "longitude", 0.0);
      srv.request.altitude = fieldDouble(service.fields(), "altitude", command.capability() == "robot.land" ? 0.0 : 2.0);
      ros::ServiceClient client = command.capability() == "robot.land" ? robot.land_client : robot.takeoff_client;
      const std::string name = command.capability() == "robot.land" ? robot.land_service : robot.takeoff_service;
      if (!client.call(srv)) {
        ROS_WARN_STREAM("XGC ROS1 adapter service failed: " << name);
      }
      return;
    }
    if (service_type == "mavros_msgs/CommandLong") {
      mavros_msgs::CommandLong srv;
      srv.request.broadcast = fieldBool(service.fields(), "broadcast", false);
      srv.request.command = static_cast<uint16_t>(fieldDouble(service.fields(), "command", 0));
      srv.request.confirmation = static_cast<uint8_t>(fieldDouble(service.fields(), "confirmation", 0));
      srv.request.param1 = fieldDouble(service.fields(), "param1", 0.0);
      srv.request.param2 = fieldDouble(service.fields(), "param2", 0.0);
      srv.request.param3 = fieldDouble(service.fields(), "param3", 0.0);
      srv.request.param4 = fieldDouble(service.fields(), "param4", 0.0);
      srv.request.param5 = fieldDouble(service.fields(), "param5", 0.0);
      srv.request.param6 = fieldDouble(service.fields(), "param6", 0.0);
      srv.request.param7 = fieldDouble(service.fields(), "param7", 0.0);
      if (!robot.command_client.call(srv)) {
        ROS_WARN_STREAM("XGC ROS1 adapter service failed: " << robot.command_service);
      }
      return;
    }
    ROS_WARN_STREAM("XGC ROS1 adapter unsupported service command type: " << service_type);
  }

  void publishTopicSignal(const xgc::adapter::v1::AdapterCommand& command) {
    ros::Publisher publisher;
    {
      std::lock_guard<std::mutex> lock(mu_);
      auto it = robots_.find(command.robot_id());
      if (it == robots_.end()) {
        return;
      }
      auto& robot = it->second;
      const auto& signal = command.topic_signal();
      if (signal.message_type() != "std_msgs/Empty") {
        ROS_WARN_STREAM("XGC ROS1 adapter unsupported topic signal type: " << signal.message_type());
        return;
      }
      if (command.capability() == "robot.takeoff" || signal.topic() == robot.takeoff_topic) {
        publisher = robot.takeoff_pub;
      } else if (command.capability() == "robot.land" || signal.topic() == robot.land_topic) {
        publisher = robot.land_pub;
      } else {
        ROS_WARN_STREAM("XGC ROS1 adapter topic signal does not match robot=" << command.robot_id()
                                                                              << " topic=" << signal.topic());
        return;
      }
    }
    if (publisher) {
      publisher.publish(std_msgs::Empty());
    }
  }

  void remoteControlTimer(const ros::TimerEvent&) {
    std::vector<std::pair<ros::Publisher, geometry_msgs::Twist>> twist_publishes;
    std::vector<std::pair<ros::Publisher, mavros_msgs::PositionTarget>> target_publishes;
    {
      std::lock_guard<std::mutex> lock(mu_);
      for (auto& entry : robots_) {
        auto& robot = entry.second;
        const bool px4_rotor = robot.robot_type == "px4" || robot.robot_type == "minidrone";
        geometry_msgs::Twist command;
        bool should_publish = false;
        if (robot.remote_enabled) {
          if (px4_rotor) {
            auto target = robot.current_position_target_cmd;
            target.header.stamp = ros::Time::now();
            target_publishes.emplace_back(robot.position_target_pub, target);
          } else {
            command = robot.current_remote_cmd;
            should_publish = true;
          }
        } else if (robot.pending_stop_publish) {
          command = geometry_msgs::Twist();
          robot.pending_stop_publish = false;
          should_publish = !px4_rotor;
        }
        if (should_publish && robot.cmd_vel_pub) {
          twist_publishes.emplace_back(robot.cmd_vel_pub, command);
        }
      }
    }
    for (const auto& item : twist_publishes) {
      item.first.publish(item.second);
    }
    for (const auto& item : target_publishes) {
      item.first.publish(item.second);
    }
  }

  void publishStopAll() {
    std::vector<ros::Publisher> publishers;
    {
      std::lock_guard<std::mutex> lock(mu_);
      for (const auto& entry : robots_) {
        if ((entry.second.robot_type != "px4" && entry.second.robot_type != "minidrone") && entry.second.cmd_vel_pub) {
          publishers.push_back(entry.second.cmd_vel_pub);
        }
      }
    }
    for (const auto& publisher : publishers) {
      publisher.publish(geometry_msgs::Twist());
    }
  }

  bool isShuttingDown() {
    std::lock_guard<std::mutex> lock(mu_);
    return shutting_down_;
  }

  void poseCallback(const std::string& robot_id, const geometry_msgs::PoseStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mu_);
    auto& robot = robots_[robot_id];
    robot.pose = *msg;
    robot.has_pose = true;
    ++robot.pose_count;
    robot.last_pose_wall = ros::WallTime::now();
  }

  void twistCallback(const std::string& robot_id, const geometry_msgs::TwistStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mu_);
    auto& robot = robots_[robot_id];
    robot.twist = *msg;
    robot.has_twist = true;
    ++robot.twist_count;
    robot.last_twist_wall = ros::WallTime::now();
  }

  void twistPlainCallback(const std::string& robot_id, const geometry_msgs::Twist::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mu_);
    auto& robot = robots_[robot_id];
    robot.twist.header.stamp = ros::Time::now();
    robot.twist.header.frame_id = "map";
    robot.twist.twist = *msg;
    robot.has_twist = true;
    ++robot.twist_count;
    robot.last_twist_wall = ros::WallTime::now();
  }

  void imuCallback(const std::string& robot_id, const sensor_msgs::Imu::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mu_);
    auto& robot = robots_[robot_id];
    robot.imu = *msg;
    robot.has_imu = true;
    ++robot.imu_count;
    robot.last_imu_wall = ros::WallTime::now();
  }

  void cmdVelCallback(const std::string& robot_id, const geometry_msgs::Twist::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mu_);
    auto& robot = robots_[robot_id];
    robot.cmd_vel = *msg;
    robot.has_cmd_vel = true;
    ++robot.cmd_vel_count;
    robot.last_cmd_vel_wall = ros::WallTime::now();
  }

  void positionTargetCallback(const std::string& robot_id, const mavros_msgs::PositionTarget::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mu_);
    auto& robot = robots_[robot_id];
    robot.cmd_vel.linear.x = msg->velocity.x;
    robot.cmd_vel.linear.y = msg->velocity.y;
    robot.cmd_vel.linear.z = msg->velocity.z;
    robot.cmd_vel.angular.z = msg->yaw_rate;
    robot.has_cmd_vel = true;
    ++robot.cmd_vel_count;
    robot.last_cmd_vel_wall = ros::WallTime::now();
  }

  static mavros_msgs::PositionTarget px4PositionTarget(const xgc::adapter::v1::RemoteControlCommand& remote) {
    mavros_msgs::PositionTarget msg;
    msg.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    msg.type_mask = mavros_msgs::PositionTarget::IGNORE_PX |
                    mavros_msgs::PositionTarget::IGNORE_PY |
                    mavros_msgs::PositionTarget::IGNORE_VZ |
                    mavros_msgs::PositionTarget::IGNORE_AFX |
                    mavros_msgs::PositionTarget::IGNORE_AFY |
                    mavros_msgs::PositionTarget::IGNORE_AFZ |
                    mavros_msgs::PositionTarget::IGNORE_YAW;
    msg.velocity.x = remote.linear_x();
    msg.velocity.y = remote.linear_y();
    msg.position.z = remote.linear_z() > 0.0 ? remote.linear_z() : 1.0;
    msg.yaw_rate = remote.angular_z();
    return msg;
  }

  void mavrosStateCallback(const std::string& robot_id, const mavros_msgs::State::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mu_);
    auto& robot = robots_[robot_id];
    robot.mavros_state = *msg;
    robot.has_mavros_state = true;
    ++robot.state_count;
    robot.last_state_wall = ros::WallTime::now();
  }

  void extendedStateCallback(const std::string& robot_id, const mavros_msgs::ExtendedState::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mu_);
    auto& robot = robots_[robot_id];
    robot.extended_state = *msg;
    robot.has_extended_state = true;
    ++robot.extended_state_count;
    robot.last_extended_state_wall = ros::WallTime::now();
  }

  void batteryCallback(const std::string& robot_id, const sensor_msgs::BatteryState::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mu_);
    auto& robot = robots_[robot_id];
    robot.battery = *msg;
    robot.has_battery = true;
    ++robot.battery_count;
    robot.last_battery_wall = ros::WallTime::now();
  }

  void globalPositionCallback(const std::string& robot_id, const sensor_msgs::NavSatFix::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mu_);
    auto& robot = robots_[robot_id];
    robot.global_position = *msg;
    robot.has_global_position = true;
    ++robot.global_position_count;
    robot.last_global_position_wall = ros::WallTime::now();
  }

  void stageCallback(const std::string& robot_id, const std_msgs::Int32::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mu_);
    auto& robot = robots_[robot_id];
    robot.stage = *msg;
    robot.has_stage = true;
    ++robot.stage_count;
    robot.last_stage_wall = ros::WallTime::now();
  }

  void angleCallback(const std::string& robot_id, const geometry_msgs::Vector3::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mu_);
    auto& robot = robots_[robot_id];
    robot.angle = *msg;
    robot.has_angle = true;
    ++robot.angle_count;
    robot.last_angle_wall = ros::WallTime::now();
  }

  void iasCallback(const std::string& robot_id, const std_msgs::Float32::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mu_);
    auto& robot = robots_[robot_id];
    robot.ias = *msg;
    robot.has_ias = true;
    ++robot.ias_count;
    robot.last_ias_wall = ros::WallTime::now();
  }

  void posCallback(const std::string& robot_id, const geometry_msgs::Vector3::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mu_);
    auto& robot = robots_[robot_id];
    robot.pos = *msg;
    robot.has_pos = true;
    ++robot.pos_count;
    robot.last_pos_wall = ros::WallTime::now();
  }

  void baroCallback(const std::string& robot_id, const std_msgs::Float32::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mu_);
    auto& robot = robots_[robot_id];
    robot.baro = *msg;
    robot.has_baro = true;
    ++robot.baro_count;
    robot.last_baro_wall = ros::WallTime::now();
  }

  void updateTopicMetrics(const ros::TimerEvent&) {
    std::lock_guard<std::mutex> lock(mu_);
    for (auto& entry : robots_) {
      auto& robot = entry.second;
      robot.pose_freq_hz = robot.pose_count;
      robot.twist_freq_hz = robot.twist_count;
      robot.imu_freq_hz = robot.imu_count;
      robot.cmd_vel_freq_hz = robot.cmd_vel_count;
      robot.state_freq_hz = robot.state_count;
      robot.extended_state_freq_hz = robot.extended_state_count;
      robot.battery_freq_hz = robot.battery_count;
      robot.global_position_freq_hz = robot.global_position_count;
      robot.stage_freq_hz = robot.stage_count;
      robot.angle_freq_hz = robot.angle_count;
      robot.ias_freq_hz = robot.ias_count;
      robot.pos_freq_hz = robot.pos_count;
      robot.baro_freq_hz = robot.baro_count;
      robot.pose_count = 0;
      robot.twist_count = 0;
      robot.imu_count = 0;
      robot.cmd_vel_count = 0;
      robot.state_count = 0;
      robot.extended_state_count = 0;
      robot.battery_count = 0;
      robot.global_position_count = 0;
      robot.stage_count = 0;
      robot.angle_count = 0;
      robot.ias_count = 0;
      robot.pos_count = 0;
      robot.baro_count = 0;
    }
  }

  void publishState(const ros::TimerEvent&) {
    std::vector<RobotRuntime> snapshots;
    {
      std::lock_guard<std::mutex> lock(mu_);
      for (const auto& entry : robots_) {
        snapshots.push_back(entry.second);
      }
    }
    xgc::adapter::v1::PushRobotStateRequest request;
    request.set_adapter_id(adapter_id_);
    request.set_experiment_id(experiment_id_);
    request.set_observed_unix_nanos(ros::WallTime::now().toNSec());
    for (const auto& robot : snapshots) {
      const auto& behavior = behaviorForType(robot.robot_type);
      if (!behavior.should_publish_state(robot)) {
        continue;
      }
      behavior.add_state(request, robot);
    }
    if (request.states_size() == 0) {
      return;
    }
    xgc::adapter::v1::PushRobotStateResponse response;
    grpc::ClientContext context;
    const auto status = stub_->PushRobotState(&context, request, &response);
    if (!status.ok()) {
      ROS_WARN_THROTTLE(5.0, "XGC adapter state push failed: %s", status.error_message().c_str());
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  std::unique_ptr<xgc::adapter::v1::AdapterIngress::Stub> stub_;
  ros::Timer publish_timer_;
  ros::Timer heartbeat_timer_;
  ros::Timer metrics_timer_;
  ros::Timer remote_timer_;
  std::thread command_thread_;

  std::mutex mu_;
  std::map<std::string, RobotRuntime> robots_;
  bool shutting_down_ = false;
  std::shared_ptr<grpc::ClientContext> active_command_context_;

  std::string adapter_id_;
  std::string experiment_id_;
  std::string socket_path_;
  double publish_rate_hz_ = 10.0;
  double remote_publish_rate_hz_ = 20.0;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "xgc_ros1_adapter");
  Ros1AdapterNode node(ros::NodeHandle(), ros::NodeHandle("~"));
  ros::spin();
  return 0;
}
