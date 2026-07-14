# XGC2 ROS1 Robot Adapters

This Catkin workspace contains two independent robot-specific AdapterLink
products. There is no generic `xgc_ros1_adapter` package or executable.

The repository is distributed under the BSD 3-Clause License in `LICENSE`.

Both processes link the ROS-independent
`libxgc2-adapter-link-client-dev` product. That library exclusively owns gRPC,
Unix-socket transport, registration, bootstrap credentials, sessions,
heartbeats, reconnects, telemetry batching, operation streaming, and operation
idempotency. This repository contains only ROS/native robot mappings.

## Product outputs

| ROS package | Debian package | Profile | Native dependency |
| --- | --- | --- | --- |
| `xgc_px4_multirotor_ros1_adapter` | `ros-noetic-xgc2-px4-multirotor-adapter` | `px4.multirotor.ros1.v1` | `mavros_msgs` |
| `xgc_scout_mini_ros1_adapter` | `ros-noetic-xgc2-scout-mini-adapter` | `scout-mini.ros1.v1` | `scout_msgs` |

The PX4 package has no Scout dependency. The Scout package has no MAVROS
dependency. The removed package is not retained as a meta package or adapter.

One process owns every same-type robot in the immutable `AdapterPlan`. For
example, a run containing three PX4 multirotors and two Scout Mini robots uses
one PX4 adapter process and one Scout adapter process, not one process per robot
and not one mixed process.

## Native mappings

### PX4 multirotor

For each plan namespace such as `/uav1`, the adapter consumes:

- `mavros/local_position/pose`
- `mavros/local_position/velocity_local`
- `mavros/imu/data`
- `mavros/battery`
- `mavros/state`
- `mavros/extended_state`

It publishes typed pose, velocity, IMU, power, health, flight, and channel
health messages. It implements only the profile's typed operations:

- `operation.arm` through `mavros/cmd/arming`
- `operation.mode` through `mavros/set_mode`
- `operation.autopilot-reboot` through `mavros/cmd/command`

### Scout Mini

For each plan namespace such as `/scout1`, the adapter consumes:

- `odom`
- `imu/data_raw`
- `scout_status`

It publishes typed pose, velocity, IMU, power, health, and channel health
messages. The current Scout profile exposes no operation channel.

Image, point-cloud, TF visualization, and other high-bandwidth data remain on
their direct ROS/Foxglove paths and are not sent through AdapterLink.

## Contract enforcement

The public protobuf product remains the single source of protocol and profile
truth. `xgc2_adapter_link_client` exports its generated protocol target and the
installed registry/profile paths. Each Catkin package generates a private,
single-profile metadata header and fails its build when:

- the selected profile digest or native endpoints change;
- an unknown channel appears;
- a channel schema/message fingerprint does not exist;
- an unsupported input channel is introduced.

The ROS packages do not invoke `protoc`, compile gRPC, vendor protobuf files, or
copy AdapterLink client/session code.

## Install

```bash
sudo apt update
sudo apt install \
  ros-noetic-xgc2-px4-multirotor-adapter \
  ros-noetic-xgc2-scout-mini-adapter
```

Install only the package needed by a robot target when the target is not a
mixed centralized-simulation environment.

## Build and test

The development environment requires ROS Noetic, both native message packages,
and the public AdapterLink client development package:

```bash
sudo apt update
sudo apt install \
  libxgc2-adapter-link-client-dev \
  ros-noetic-mavros-msgs \
  ros-noetic-nav-msgs \
  ros-noetic-scout-msgs \
  python3-yaml

source /opt/ros/noetic/setup.bash
catkin_make
catkin_make run_tests
catkin_test_results --verbose build/test_results
```

The release path builds and install-checks both independent Debian packages:

```bash
.xgc2/scripts/build_debs_in_docker.sh --output-dir "$PWD/debs"
```

CI bootstraps its common dependencies from the public pinned tags
`xgc2-protobuf@v0.2.0-1` and
`xgc2-adapter-link-client-cpp@v0.1.0-1`, builds their Debian packages inside
the clean container, and installs those packages before building this
workspace. It therefore does not depend on a new production APT publication
having completed. Set `XGC2_BOOTSTRAP_COMMON_FROM_GIT=false` to exercise the
already-published APT path instead. `XGC2_APT_OVERLAY_URL` remains available
for release-train dependencies.

## Runtime

Go Core supplies a process-specific adapter ID and bootstrap token file.

```bash
rosrun xgc_px4_multirotor_ros1_adapter \
  xgc_px4_multirotor_ros1_adapter_node \
  _adapter_id:=px4-run-123 \
  _socket_path:=/run/xgc2/adapter/adapter-link.sock \
  _bootstrap_token_file:=/run/xgc2/adapter/px4-run-123.token

rosrun xgc_scout_mini_ros1_adapter \
  xgc_scout_mini_ros1_adapter_node \
  _adapter_id:=scout-run-123 \
  _socket_path:=/run/xgc2/adapter/adapter-link.sock \
  _bootstrap_token_file:=/run/xgc2/adapter/scout-run-123.token
```

Installed executable paths are fixed:

```text
/opt/ros/noetic/lib/xgc_px4_multirotor_ros1_adapter/xgc_px4_multirotor_ros1_adapter_node
/opt/ros/noetic/lib/xgc_scout_mini_ros1_adapter/xgc_scout_mini_ros1_adapter_node
```

Robot IDs, namespaces, channels, and profile digests come exclusively from the
Core `AdapterPlan`; neither package has a local robot inventory fallback.
