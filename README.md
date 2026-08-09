# XGC2 ROS1 Robot Adapters

This Catkin workspace contains four robot-domain Adapter Runtime applications.
An Adapter is a general capability plugin for Core or Agent; these applications
specialize that abstraction for PX4 multirotors, Scout Mini robots, Mecanum
UGVs, and the read-only Unitree B2 data plane.

| ROS package | Debian package | Provider definition | Robot profile |
| --- | --- | --- | --- |
| `xgc_px4_multirotor_ros1_adapter` | `ros-noetic-xgc2-px4-multirotor-adapter` | `xgc2-px4-multirotor-ros1-adapter` | `px4.multirotor.ros1.v7` |
| `xgc_scout_mini_ros1_adapter` | `ros-noetic-xgc2-scout-mini-adapter` | `xgc2-scout-mini-ros1-adapter` | `scout-mini.ros1.v6` |
| `xgc_mecanum_ugv_ros1_adapter` | `ros-noetic-xgc2-mecanum-ugv-adapter` | `xgc2-mecanum-ugv-ros1-adapter` | `mecanum-ugv.ros1.v3` |
| `xgc_unitree_b2_ros1_adapter` | `ros-noetic-xgc2-unitree-b2-adapter` | `xgc2-unitree-b2-ros1-adapter` | `unitree.b2.v1` |

The generic C++ Adapter Runtime SDK owns registration, trusted bootstrap,
session fencing, capability dispatch, flow control, reconnects, and terminal
result delivery. This repository owns robot semantics, ROS1-native mappings,
profile contracts, and native safety policy.

## Runtime contract

Core resolves each provider from the installed robot profile catalog. A
provider instance uses a `robot-group` scope containing `target-id`, `run-id`,
and `provider`; each invocation and telemetry source uses a separate
`robot-resource` subject containing `target-id`, `run-id`, and `robot-id`.

All four applications expose:

- `xgc.robot.telemetry@1`: `telemetry` source of serialized
  `xgc.robot.v1.RobotMessage`

The PX4 product also exposes `xgc.robot.command@1`:

- `arm` with `xgc.semantic.aerial.v1.ArmRequest`
- `set-flight-mode` with `xgc.semantic.aerial.v1.ModeRequest`
- `reboot-autopilot` with `xgc.semantic.aerial.v1.AutopilotRebootRequest`

PX4 command operations require deadlines and idempotency keys. A successful
operation returns the registry-owned `xgc.v1.Empty` payload. Native ROS service
calls are not advertised as cancellable after dispatch.

The installed Profile catalog owns each operation's closed JSON parameter
schema and timeout. The flight-mode enum is projected directly from the PX4
source Profile's `policy.allowed_modes`, and every Profile timeout is generated
from the same `policy.timeout_ms` used by its provider endpoint.

## Native mappings

PX4 telemetry and diagnostics consume MAVROS topics under each configured
robot namespace. Mocap samples from
`/vrpn_client_node/{mocap_rigid_body}/pose` are relayed to
`mavros/vision_pose/pose` without coordinate transformation, interpolation, or
stale-sample repetition. Arm, flight-mode, and autopilot-reboot operations use
typed MAVROS services. Flight modes are restricted by the source Profile's
single native allowlist; reboot requires a known, fresh, connected, disarmed
vehicle state.

Scout Mini telemetry consumes the global
`/vrpn_client_node/{mocap_rigid_body}/pose` and `twist` streams plus `cmd_vel`,
`imu/data_raw`, and `scout_status` under the configured namespace.
`vrpn.position` records position and orientation, while `vrpn.velocity`
retains the raw linear and angular vectors. The Adapter combines both VRPN
streams to project world-frame linear velocity onto the signed body X axis;
that processed scalar is `vrpn.speed`, so lateral slip is excluded.
`command.velocity` separately records the commanded
linear and angular velocity; the Adapter does not consume odometry. `state.power.voltageV`
retains the measured chassis voltage. `state.power.percentage` is currently a
clamped linear projection from
the Scout Mini manual's 20.5 V protection voltage (0%) to 29.2 V full-charge
voltage (100%); it is a display estimate rather than a battery state-of-charge
model.

Scout Mini also exposes the idempotent `set-motion-intent` operation with a
three-field `xgc.semantic.ground.v1.MotionIntentRequest`: `gear` is 1, 2, or 3,
while `longitudinal` and `yaw` are each -1, 0, or 1. The Adapter maps the three
gears to 0.5/1.0/1.5 m/s and approximately 0.1745/0.3490/0.5235 rad/s, clamps
the generated `geometry_msgs/Twist` to the Scout SDK limits, and republishes
the latest intent to the profile-owned namespaced `cmd_vel` topic at 10 Hz.
The caller only sends state changes; a zero intent is published immediately.
The intent remains active until the next change or until the robot source,
instance spec, or Adapter Runtime session is closed, at which point the
Adapter publishes a final zero command. Before the first motion operation the
Adapter does not publish `cmd_vel`, so an enabled but unused command channel
does not take control away from another ROS controller. `command.velocity`
observes the same topic for telemetry. Real-robot bringup must place
`scout_base_node` in that same namespace and subscribe to relative `cmd_vel`
(or explicitly remap its legacy absolute `/cmd_vel` subscription); the Adapter
does not publish a second global command topic because that would couple
multiple Scout robots.

After the first accepted motion intent, the namespaced `cmd_vel` topic must
have one effective command owner. Stop any autonomous controller first, or put
both sources behind an explicit ROS command mux; competing publishers would
otherwise interleave velocity commands.

The Mecanum UGV profile intentionally contains only `vrpn.position`, raw
`vrpn.velocity`, processed `vrpn.speed`, `command.velocity`, the existing
longitudinal/yaw motion-intent operation, and channel diagnostics. Online state
depends only on a fresh `vrpn.position`; it has no Scout status or odometry
dependency. The scalar speed is computed in C++ by projecting the world-frame
VRPN linear vector onto the body X axis from the VRPN pose quaternion, preserving
reverse sign while excluding lateral slip. Motion intent keeps the existing
protobuf contract (there is no lateral field) and maps its three gears to the
deployed SSS Mecanum limits: 0.5/1.0/1.5 m/s longitudinal and approximately
0.5236/1.0472/1.5708 rad/s yaw. Before the first accepted intent, the adapter
publishes no `cmd_vel`; afterward it republishes the latest intent at 10 Hz and
sends a final zero on shutdown.

High-bandwidth images, point clouds, and TF visualization remain on their
native ROS visualization paths rather than the semantic telemetry source.

The Unitree B2 Adapter is telemetry-only. It listens on the profile-bound TCP
endpoint, accepts only `xgc2/{robot_id}/up/*` keys from the frozen G3 JSON
contract, and has no command capability or `down/cmd` handler. One validated
sample is used for both outputs: Adapter Runtime receives `state.pose`,
`state.velocity`, `state.speed`, `state.power`, `state.health`,
`state.locomotion`, bounded leg/arm joints, and link/stream diagnostics; local
ROS receives `/<slot>/odom`, namespaced dedicated leg/arm joints, merged
`/<slot>/joint_states`, `/<slot>/path`, and
`world -> <slot>/b2_description` TF. The immutable Experiment namespace is
delivered through the same Adapter Profile contract as the wire endpoint.

B2 online state uses ground receive monotonic time, not the onboard clock. The
required windows are odom/joints 1 s, power 2 s, and driver/heartbeat 3 s. An
absent Domain-17 arm never takes the Domain-0 B2 body offline. The current C++
build implements the explicit LAB TCP backend; selecting `zenoh` fails closed
until a supported C/C++ Zenoh client is linked.

The Adapter Debian deliberately does not ship a B2-only description launch or
depend on `robot_state_publisher`/`b2arx_description`. After the frozen Run
roster selects any Robot, the separate generic `xgc2_robot_visualization`
runtime resolves that contribution's installed description, publishes
`/<slot>/visual_robot_description` plus the namespaced state-publisher
parameter, and supervises the standard ROS1
`robot_state_publisher`. It does not open the wire transport or decode a second
copy of B2 data. Opening Lichtblick never starts either process: the Experiment
Session owns Adapter, descriptions, RSP and Foxglove lifecycles together.

## Trust and installation metadata

Each Debian package owns three generated, immutable installation contracts:

- `/usr/share/xgc2/adapter-definitions/<provider>.json`
- `/usr/share/xgc2/process-definitions/<provider>.json`
- `/usr/share/xgc2/robot-adapter-profiles/<provider>.json`

The install step hashes the final ELF, computes canonical capability and public
Profile contract digests, and validates every message ID/version/fingerprint
against `xgc2-protobuf`. The process definition
accepts only the supervisor-owned `adapterBootstrapFile` parameter and invokes
the executable directly with `--adapter-bootstrap-file` and the complete ROS
Noetic runtime environment. It never relies on a shell or a sourced setup file.

Package-local C++ headers are implementation details used only while building
each executable. The Debian packages intentionally export no Catkin header or
library interface.

The application accepts no socket, token, identity, inventory, or ROS-parameter
fallback. The binary bootstrap is owner-only mode `0600` and contains the exact
initial instance specification and granted capability contracts.

## Build and test

```bash
sudo apt update
sudo apt install \
  libxgc2-adapter-runtime-client-dev \
  xgc2-protobuf-dev \
  ros-noetic-geometry-msgs \
  ros-noetic-mavros-msgs \
  ros-noetic-scout-msgs \
  ros-noetic-nav-msgs \
  ros-noetic-tf2-ros \
  nlohmann-json3-dev \
  python3-jsonschema \
  python3-yaml

python3 -m unittest discover -v -s test -p 'test_*.py'

source /opt/ros/noetic/setup.bash
catkin_make
catkin_make run_tests
catkin_test_results --verbose build/test_results
```

The release path builds and install-checks all four independent Debian packages:

```bash
.xgc2/scripts/build_debs_in_docker.sh --output-dir "$PWD/debs"
```

After publishing, the public Focal APT gate installs the exact frozen B2
version in a clean ROS Noetic container and rejects source-tree paths in every
installed runtime manifest:

```bash
.xgc2/scripts/check_public_apt_install.sh
```

All adapters are compiled against the exact
`libxgc2-adapter-runtime-client-dev` and `xgc2-protobuf-dev` inputs. Their
installed Debian packages deliberately omit those build-only dependencies:
`dpkg-shlibdeps` derives a lower-bounded
`libxgc2-adapter-runtime-client2` dependency from the ELF SONAME, while the ROS
message packages remain explicit runtime dependencies. Compatible ABI-1 SDK
updates can therefore be compatibility-verified without republishing these
adapters. An ABI break must use a new SONAME/runtime package and rebuild them.

## Supervisor launch

The Process Supervisor starts the fixed installed executable directly:

```text
/opt/ros/noetic/lib/xgc_px4_multirotor_ros1_adapter/xgc_px4_multirotor_ros1_adapter_node
/opt/ros/noetic/lib/xgc_scout_mini_ros1_adapter/xgc_scout_mini_ros1_adapter_node
/opt/ros/noetic/lib/xgc_mecanum_ugv_ros1_adapter/xgc_mecanum_ugv_ros1_adapter_node
/opt/ros/noetic/lib/xgc_unitree_b2_ros1_adapter/xgc_unitree_b2_ros1_adapter_node
```

For a diagnostic manual launch, pass a real supervisor-generated bootstrap:

```bash
rosrun xgc_px4_multirotor_ros1_adapter \
  xgc_px4_multirotor_ros1_adapter_node \
  --adapter-bootstrap-file /run/xgc2/adapter/processes/<instance>.bootstrap
```

The repository is distributed under the BSD 3-Clause License in `LICENSE`.
