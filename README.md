# ROS1 services

ROS1 processes of an XGC2 Run: the robot type servers for PX4 multirotors, Scout Mini
and Mecanum UGVs, the typed ROS1 Tools service and the scene authoring adapter.

Every process here is **Run-owned**: the experiment workflow starts it for one Run,
bound to the Run's ROS master, and ends it with the Run. Nothing outlives a Run and no
process is shared between Runs. There is one type server per robot type and ROS master;
it serves every robot of that type in the Run.

| Process | Executable | Process definitions | Service |
|---|---|---|---|
| PX4 multirotor type server | `xgc_px4_multirotor_ros1_adapter_node` | `xgc2-px4-multirotor-ros1-adapter` | `xgc2.robot-adapter`, grpc.v1 |
| Scout Mini and Mecanum UGV type servers | `xgc_ugv_ros1_adapter_node` | `xgc2-scout-mini-ros1-adapter`, `xgc2-mecanum-ugv-ros1-adapter` | `xgc2.robot-adapter`, grpc.v1 |
| ROS1 Tools | `xgc_ros1_tools_adapter_node` | `xgc2-ros1-tools-adapter` | `ros1.tools`, http.v1 |
| ROS1 Scene | `xgc2_scene_adapter` | `xgc2-ros1-scene-adapter` | `xgc2.scene.authoring` v1, http.v1 |

The UGV executable is one runtime for both chassis types; the process definition picks
the type with `--provider`. Everything that differs between Scout Mini and Mecanum
(velocity limits, sideways motion, health source, battery filter) is configuration in
`src/ugv_ros1_adapter/src/variant.cpp`.

The scene adapter validates, edits and saves `xgc2.scene.v1` documents and publishes
snapshots, state, markers and TF. Its `gazebo` profile applies scenes and motion through
the world host, discovering its instance from `simulationEndpoint`, the session's
world Unix socket path. Gazebo requires a non-empty absolute path; the world may
appear later or restart, with the latest scene replayed once per instance.
`xsim` reads, edits and saves documents with live simulator edits reporting
`unsupported`; `physical` provides authoring and visualization. Simulators own their
sensor implementations. The finite `xgc2-scene-assemble` and `xgc2-scene-prepare` tools
provide assembly and world preparation to workflows.
Gazebo preparation defaults `scene.parameters.required_components` to `["vrpn"]`;
an explicit list, including an empty list, wins. The [finite CLI documentation](docs/scene-adapter.md#finite-cli-usage)
also describes the error for the removed chassis roster parameter.

## Documents

- [Type server lifecycle and RPC contract](docs/type-servers.md): flags, exit status,
  Describe, ApplyMembers, RemoveMembers, Execute, SubscribeStatus, Health, Shutdown,
  the drain, and what the server does with the ROS master.
- [Process definitions](docs/process-definitions.md): the files the packages install and
  how a Run uses them, including the Tools and Scene services.
- [Scene adapter](docs/scene-adapter.md): profiles, private ROS parameters, topics,
  TF, authoring routes and finite CLI usage.
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
src/scene_ros1_adapter/            scene authoring, ROS visualization and finite CLIs (http.v1)
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

Packages are built with catkin against the XGC2 foundation (XRPC 0.2.0,
`xgc2-protobuf-dev` 0.7, Math 0.6.0, c-ares, gRPC 1.16),
`xgc2_geometry_msgs`, nlohmann-json, yaml-cpp and OpenSSL:

```
.xgc2/scripts/build_debs_in_docker.sh --jobs 2
```

builds in the release image, runs the tests and checks the installed packages. A local
build is `catkin_make --source src --directory <build>`; when `xgc2-protobuf-dev` is not
installed, pass `-DXGC2_PROTOBUF_PROTO_ROOT=<proto directory>` and
`-DXGC2_PROTOBUF_REGISTRY_JSON=<registry.json>` of the protocol checkout, and make the c-ares
development files visible on `CMAKE_PREFIX_PATH`. [Tests](docs/testing.md) says how to run
what the build produces.

Debian packages: `ros-noetic-xgc2-ros1-adapter-common` (the shared libraries),
`ros-noetic-xgc2-px4-multirotor-adapter`, `ros-noetic-xgc2-ugv-adapter`,
`ros-noetic-xgc2-ros1-tools-adapter` and `ros-noetic-xgc2-scene-adapter`. The first is a
dependency of the type server packages. The product version is `0.7.0-1`.
