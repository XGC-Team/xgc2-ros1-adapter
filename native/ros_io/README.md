# Lightweight vehicle live check

`lightweight_vehicle_live.py` checks the ROS/Host boundary and the native FS150, Scout, and Mecanum lightweight vehicle models. It covers their ROS state/command topics, simulated MAVROS arm/mode services, motion, and pose/velocity timestamp alignment. It does not run a controller, planner, SITL, or Gazebo.

Source the ROS environment and prepare a reachable ROS master before running. The script does not start `roscore` or Docker. All four arguments are required; the host and plugin paths are resolved to absolute paths, and the output directory is created if needed.

Run from the `sync-runtime` directory with the built artifacts:

```sh
python3 plugins/ros-io/lightweight_vehicle_live.py \
  --host /absolute/path/to/xgc-rt-host \
  --plant /absolute/path/to/liblightweight_vehicle.so \
  --ros-io /absolute/path/to/libros_io.so \
  --output-dir /absolute/path/to/lightweight-vehicle-live-output
```

The output directory receives the generated manifest, Host log, model audit data, and `ros-model-result.json`.

An optional `sim_odometry_topic` publishes MAVROS-compatible `nav_msgs/Odometry`
from the existing measured `sim_pose` and `sim_velocity` channels. It pairs only
equal, increasing model timestamps. The pose uses `frame_id`; measured world
velocity is rotated into `sim_odometry_child_frame` (default `base_link`). It
does not differentiate positions or expose a new ABI port. The live check also
checks same-step stamps, measured poses and body twists at zero and 90-degree
yaw. `bash plugins/ros-io/run-odometry-test.sh` checks coordinate and rejection
cases without ROS; neither test certifies an algorithm experiment.

`sim_imu` always carries body-frame specific force and angular velocity, with
`sim_odometry_child_frame` as its ROS header frame (default `base_link`). Set
`sim_imu_orientation_from_pose = true` for the MAVROS `/imu/data` edge to attach
the full quaternion from an exactly matching, increasing `sim_pose` timestamp.
Different model steps are never paired. The default is `false`, preserving
the unknown-orientation convention for `/imu/data_raw` and stand-alone IMU
inputs; no quaternion is added to the existing `xgc.imu/1` wire schema.
The odometry test entry also checks IMU pose pairing and invalid samples.

The optional `attitude_target_full_topic` input publishes
`attitude_target_full` with schema `xgc.attitude_target/2`: full quaternion,
body rates, normalized thrust and the original MAVROS type mask. The existing
`attitude_target` port and `/1` layout remain unchanged for hover-thrust
observers. The new port preserves even ignored payload fields; an actuator
consumer must explicitly interpret or reject unsupported masks.
`bash plugins/ros-io/run-attitude-target-test.sh` checks all 256 mask values
using the installed MAVROS message type. This input conversion alone does not
provide NMPC/DFBC actuator control in the lightweight plant.

Each step services the plugin's ROS queue once without blocking and then
keeps servicing it, in waits of at most 1 ms, for `slice_ms` (never past 1 ms
before the round's deadline; without `slice_ms`, until then). A remainder
shorter than the kernel's 50 us timer slack is not waited, so the lightweight
plant's `slice_ms = 0.001` steps are one non-blocking pass.
`bash plugins/ros-io/run-slice-test.sh` checks this arithmetic without ROS.

## Lightweight controller live check

`lightweight_controller_live.py` runs the FS150 plant and the real ctl-px4 SMC controller in separate `xgc-rt-host` processes. It sends the plant/controller channels over Zenoh TCP and uses the existing ROS master only for test commands, PVA setpoints, and observed ROS outputs. Source the ROS environment and prepare a reachable ROS master first; this script does not start ROS, MAVROS, or Docker. It checks the existing takeoff, 10-second trajectory tracking, endpoint error, and landing sequence. It does not validate DMPC or a complete experiment.

Run from the `sync-runtime` directory with the built artifacts and two unused loopback TCP ports:

```sh
python3 plugins/ros-io/lightweight_controller_live.py \
  --host /absolute/path/to/xgc-rt-host \
  --plant /absolute/path/to/liblightweight_vehicle.so \
  --controller /absolute/path/to/libctl_px4.so \
  --ros-io /absolute/path/to/libros_io.so \
  --plant-endpoint tcp/127.0.0.1:17441 \
  --controller-endpoint tcp/127.0.0.1:17442 \
  --output-dir /absolute/path/to/lightweight-controller-live-output
```

The output directory receives separate plant and controller manifests, Host logs, audit directories, and `controller-live-result.json` with both process IDs and exit codes.

## Batched simulation edge (`ros_sim_edge`)

`ros_sim_edge.cpp` builds the plugin `ros-sim-edge` (second output of
`build.sh OUTPUT.so SIM_EDGE_OUTPUT.so`). It serves up to six simulated robots
of one lightweight-plant batch from one module thread: per robot and role it
runs the unchanged `RosIo` of `ros_io.cpp` (the file is included as it is, only
its plugin entry point is left out), so topics, types, frames, mocap noise
streams, simulated MAVROS services and provider generations are those of the
per-robot edges. What is shared is the thread, one non-blocking ROS pass per
step and one read of the plant inputs (`sim_edge_batch.hpp`, `Tee`).

Roles of one robot: `mavros` (the MAVROS-facing edge: map frame, FCU state,
services, provider) and `mocap` (the simulated mocap source: world frame,
measurement noise; for a ground robot it also takes `cmd_vel`). Both roles
read the same plant samples. Ports are a stride-9 block per robot (`sim_pose`,
`sim_velocity`, `sim_imu`, `sim_fcu_state`, `sim_attitude_target`,
`alg_setpoint`, `sim_fcu_request`, `attitude_target_full`, `cmd_vel`; block r > 0
appends `_r`) followed by the four shared batch ports `sim_fcu_result`,
`sim_extended_state`, `sim_provider_request`, `sim_provider_result`: 58 of 64.
A port is carried exactly when its role configured the topic. Config keys are
`r<k>_<role>_<key>` for robot block k; all other keys are global.

A failed step or module output write faults the instance, that is the robots of
one batch (at most six), where a per-robot edge faulted its own one or two
edges. `bash run-sim-edge-batch-test.sh` checks the layout, the config split,
the tee and the per-robot host without ROS. `plant_graph_live.py` runs the real
Host with a Core-generated manifest and prints the ROS graph the plant creates,
to compare the per-robot and the batched layout.
