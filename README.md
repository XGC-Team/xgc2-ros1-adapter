# ROS1 services

ROS1 processes of an XGC2 Run: the robot type servers for PX4 multirotors, Scout Mini
and Mecanum UGVs, and the typed ROS1 Tools service.

Every process here is **Run-owned**: the experiment workflow starts it for one Run,
bound to the Run's ROS master, and ends it with the Run. Nothing outlives a Run and no
process is shared between Runs. There is one type server per robot type and ROS master;
it serves every robot of that type in the Run.

| Process | Executable | Process definitions | Service |
|---|---|---|---|
| PX4 multirotor type server | `xgc_px4_multirotor_ros1_adapter_node` | `xgc2-px4-multirotor-ros1-adapter` | `xgc2.robot-adapter`, grpc.v1 |
| Scout Mini and Mecanum UGV type servers | `xgc_ugv_ros1_adapter_node` | `xgc2-scout-mini-ros1-adapter`, `xgc2-mecanum-ugv-ros1-adapter` | `xgc2.robot-adapter`, grpc.v1 |
| ROS1 Tools | `xgc_ros1_tools_adapter_node` | `xgc2-ros1-tools-adapter` | `ros1.tools`, http.v1 |

The UGV executable is one runtime for both chassis types; the process definition picks
the type with `--provider`. Everything that differs between Scout Mini and Mecanum
(velocity limits, sideways motion, health source, battery filter) is configuration in
`src/ugv_ros1_adapter/src/variant.cpp`.

## Documents

- [Type server lifecycle and RPC contract](docs/type-servers.md): flags, exit status,
  Describe, ApplyMembers, RemoveMembers, Execute, SubscribeStatus, Health, Shutdown,
  the drain, and what the server does with the ROS master.
- [Process definitions](docs/process-definitions.md): the files the packages install and
  how a Run uses them, including the Tools service.
- [Tests](docs/testing.md): unit tests, generator tests and the integration tests
  against a private ROS master.

## Layout

```
src/common/                 shared code, built once as libraries
                              libxgc2_robot_server       ROS-agnostic server core
                              libxgc2_robot_server_ros1  RosBinding for ROS1, bounded master calls
src/px4_multirotor_ros1_adapter/   PX4 robots: profiles, telemetry, arm/mode/reboot, motion
src/ugv_ros1_adapter/              Scout Mini and Mecanum robots: one runtime, two variants
src/ros1_tools_adapter/            ROS1 Tools service (http.v1)
profiles/ros1/              robot profiles (YAML) the contracts are generated from
tools/                      contract and profile catalog generators and verifiers
test/                       generator and process definition tests, integration tests
.xgc2/                      product metadata and Debian packaging
```

The server core never includes ROS headers. Everything it needs from the middleware is
the `RosBinding` interface in `robot_server.hpp`; `Ros1Binding` implements it for roscpp.
A robot is a `NativeRobot` created by a factory, with a handle to its ROS node, and owns no
thread: the core calls it from fixed threads.

## Build

Packages are built with catkin against the XGC2 foundation (XRPC C++17 libraries,
`xgc2-protobuf-dev` 0.7, c-ares, gRPC 1.16):

```
.xgc2/scripts/build_debs_in_docker.sh
```

builds in the release image, runs the tests and checks the installed packages. A local
build is `catkin_make --source src` with `XGC2_PROTOBUF_PROTO_ROOT` and
`XGC2_PROTOBUF_REGISTRY_JSON` pointing at the protocol checkout when `xgc2-protobuf-dev`
is not installed.

Debian packages: `ros-noetic-xgc2-ros1-adapter-common` (the shared libraries),
`ros-noetic-xgc2-px4-multirotor-adapter`, `ros-noetic-xgc2-ugv-adapter` and
`ros-noetic-xgc2-ros1-tools-adapter`. The first is a dependency of the type server packages.
