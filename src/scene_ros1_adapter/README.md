# xgc_scene_ros1_adapter

The Run-owned scene authoring adapter exposes `xgc2.scene.authoring` v1 over
XRPC `http.v1` on a Unix socket and `xgc2.sensor.lidar` v1 on a second required
socket (`~lidar_xrpc_socket`). `xgc2_scene_adapter` publishes scene snapshots,
state, envelope JSON, markers and TF. Profiles are `gazebo`, `xsim` and `physical`.
The finite tools are `xgc2-scene-assemble` and `xgc2-scene-prepare`.
The Gazebo profile hosts the CPU fleet; `xsim` and `physical` host an empty lidar
service. Both services answer unbound readiness discovery before the ROS master
starts, report `ros_master: "waiting"` and wait without a startup deadline.
Readiness reflects the adapter's own resources. Gazebo world availability, poses,
scene truth and subscribers are runtime inputs. Edits remain usable during a
world outage; the client lazily discovers replacement instances and replays the
latest scene once, with native synchronization reported separately.
Core supplies the world rendezvous path from `$session.endpoints.world` through
`simulationEndpoint` (`~simulation_endpoint`). Gazebo requires a non-empty absolute
Unix socket path; other profiles ignore it. The adapter discovers the instance
itself and uses `targetId` to fence it to the local target.

[Scene adapter](../../docs/scene-adapter.md) documents the profiles, private ROS
parameters, topics, TF and CLI usage. The Debian package is
`ros-noetic-xgc2-scene-adapter`; its process definition is
`process/xgc2-ros1-scene-adapter.json`.

`xgc_scene_core` (`src/core`, `include/xgc_scene`) is the ROS-free static authoring
library: document validation, YAML/JSON, motion, assembly, generation, preparation
and persistence. It uses nlohmann-json, yaml-cpp, OpenSSL and `xgc2_math::scene`
(Math 0.6.0 or newer).

`xgc_scene_lidar` owns the ROS-free fleet schedules, worker pool and scan buffers.
`xgc_scene_lidar_ros` (`src/lidar_ros`, `include/xgc_scene/lidar_ros`) adds pose
subscriptions, subscriber demand, reused PointCloud2 messages and one ROS timer.
It taps simulator-applied scene truth inside the authoring node and preserves
the source enable control. See [Lidar fleet](../../docs/scene-adapter.md#lidar-fleet)
for the stdin session contract, defaults and topics. Core copies frozen session
facts unchanged and computes no fleet manifest. With `--session-stdin`, the
adapter reads the `sessionJson` document (at most 8 MiB) before startup and builds
the fleet from the robot roster. Sensor clouds use `<namespace>/simple_lidar/points`
and requested beams use `<namespace>/simple_lidar/beams`; the reference cloud
remains latched on `/xgc/scene/reference_cloud`. Old node-level preset YAML,
topic patterns and file provisioning are not carried over.

Core sources (`src/core/*.cpp`) and tests (`test/core/*_test.cpp`) are globbed.
The golden corpus (`test/scene_golden` at the repository root) is the reference.

```sh
cmake -S src/scene_ros1_adapter -B build -DCMAKE_PREFIX_PATH=/path/to/math-prefix
cmake --build build -j2
(cd build && ctest --output-on-failure -j2)
```
