# XGC2 ROS1 Robot Adapters

This Catkin workspace contains five robot-domain adapters
and one separately deployable onboard Forwarder. An Adapter is a general
capability plugin for Core or Agent; these applications specialize that
abstraction for PX4 multirotors, Scout Mini robots, Mecanum UGVs, the read-only
Unitree B2 data plane, and Mocap Rotor telemetry. The Forwarder is not an
Adapter Runtime application and is installed only on the Mocap Rotor Orin NX.

| ROS package | Debian package | Provider definition | Robot profile |
| --- | --- | --- | --- |
| `xgc_px4_multirotor_ros1_adapter` | `ros-noetic-xgc2-px4-multirotor-adapter` | `xgc2-px4-multirotor-ros1-adapter` | `px4-multirotor.physical.vrpn` |
| `xgc_scout_mini_ros1_adapter` | `ros-noetic-xgc2-scout-mini-adapter` | `xgc2-scout-mini-ros1-adapter` | `scout-mini.physical.vrpn` |
| `xgc_mecanum_ugv_ros1_adapter` | `ros-noetic-xgc2-mecanum-ugv-adapter` | `xgc2-mecanum-ugv-ros1-adapter` | `mecanum-ugv.physical.vrpn` |
| `xgc_unitree_b2_ros1_adapter` | `ros-noetic-xgc2-unitree-b2-adapter` | `xgc2-unitree-b2-ros1-adapter` | `unitree.b2.v1` |
| `xgc_mocap_rotor_ros1_adapter` | `ros-noetic-xgc2-mocap-rotor-adapter` | `xgc2-mocap-rotor-ros1-adapter` | `px4.mocap-rotor.ros1.v1` |
| `xgc_mocap_rotor_zenoh_forwarder` | `ros-noetic-xgc2-mocap-rotor-forwarder` | `xgc2-mocap-rotor-link` | onboard process only |

PX4, Scout Mini and Mecanum each run one lightweight server per provider and
actual ROS master environment. Core uses a private UDS and the protobuf
`RobotAdapterServer` service to batch member updates, invoke operations and
receive current state. The Process Supervisor still owns process lifecycle.
Unitree B2 and Mocap Rotor retain the C++ Adapter Runtime Link protocol.

Each of these three providers installs three source profiles. A server can
hold members from all three profiles at once:

| Source | PX4 | Scout Mini | Mecanum |
| --- | --- | --- | --- |
| Physical VRPN | `px4-multirotor.physical.vrpn` | `scout-mini.physical.vrpn` | `mecanum-ugv.physical.vrpn` |
| Gazebo VRPN | `px4-multirotor.gazebo.vrpn` | `scout-mini.gazebo.vrpn` | `mecanum-ugv.gazebo.vrpn` |
| xsim ROS | `px4-multirotor.xsim.ros` | `scout-mini.xsim.ros` | `mecanum-ugv.xsim.ros` |

Physical members subscribe to the selected tracker and apply the world offset.
Gazebo members subscribe to the slot's simulated tracker with zero additional
offset. Both retain the original ROS publication and PX4 vision behavior.
xsim members subscribe directly to `localization_pose_topic` and
`localization_twist_topic`, normally `/<slot>/pose` and `/<slot>/twist`, and use
these measurements for instruments and positioning health. They create no
VRPN subscription, pose/twist forwarding publisher or vision publisher. xsim
FS150 retains MAVROS state/local-position/IMU inputs. xsim Scout and Mecanum
retain the original IMU subscriptions and online conditions; position gates
operational readiness. Their simulator publishes IMU on the vehicle's original
ROS interface.

## Runtime contract

The three type servers use `robot-server` scope (`target-id`, `provider`,
`ros-master-uri`). Neither Run IDs nor robot membership affect process identity.
Individual member routes retain target, Run, robot and connection epoch. Native
ROS resources are shared only with compatible configuration; removing one
route leaves other uses active.

Each server has five application threads: one gRPC completion queue, two ROS
callback workers, one command event loop and one registration/cleanup worker.
ROS and gRPC library threads are counted separately. Members own data and ROS
handles, without their own process, helper, executor or worker thread.
The callback workers take one callback at a time from the server's two shared
ROS queues. Subscriptions request TCP_NODELAY and retain their original bounded
queue sizes. Each member's callbacks stay on one of the two shared workers,
avoiding two workers contending for a single member's mutex. A burst fills that
topic's queue rather than creating more workers
or retaining an unbounded history; member registration and command waits do not
occupy these callback workers.

All five providers expose `xgc.robot.telemetry@1` with serialized
`xgc.robot.v1.RobotMessage`. Original native subscribers, canonical publishers,
coordinate offsets, vision injection and command mappings remain in their
robot-specific processors. Status streams keep current values by channel and
coalesce slow consumers; canonical ROS publication follows its original cadence.

PX4 exposes `arm`, `set-flight-mode` and `reboot-autopilot`. Its shared event loop
can keep multiple native TCPROS requests in flight. Discovery, DNS, connection,
request and response share an absolute deadline, and cancellation closes the
actual I/O. Side effects for one robot execute in order. A timeout or disconnect
after request bytes may have reached ROS returns uncertain and blocks subsequent
conflicting commands. Core records dispatch and results in the existing operation
attempt ledger; reconnect never replays commands. Success retains the registered
`xgc.v1.Empty` return contract.

The installed Profile catalog owns each operation's closed JSON parameter
schema and timeout. The flight-mode enum is projected directly from the PX4
source Profile's `policy.allowed_modes`, and every Profile timeout is generated
from the same `policy.timeout_ms` used by its provider endpoint.

## Native mappings

PX4 telemetry and diagnostics consume MAVROS topics under each configured
robot namespace. The Adapter also owns this slot's selected raw VRPN source:
it applies the run-frozen XYZ offset once and publishes canonical
`/{namespace}/{pose,twist,accel}`. The same pose callback publishes
`/{namespace}/mavros/vision_pose/pose` through the XGC1 recent-five-publish
window at a **30 Hz** target—not 50 Hz and never cache+timer. Actual vision
publish events produce `state.vision.pose.source_rate_hz` for HUD VIS. Arm,
flight-mode, and autopilot-reboot operations use
typed MAVROS services. Flight modes are restricted by the source Profile's
single native allowlist; reboot requires a known, fresh, connected, disarmed
vehicle state.

Scout Mini Adapter consumes the run-selected raw VRPN pose/twist/accel,
applies the offset once, and uniquely publishes namespaced
`/{namespace}/pose` (`geometry_msgs/PoseStamped`), `/{namespace}/twist`
(`geometry_msgs/TwistStamped`), and `/{namespace}/accel`
(`geometry_msgs/AccelStamped`). It also consumes `cmd_vel`,
`imu/data_raw`, `PowerVoltage`, and `scout/chassis_state` under the configured namespace.
`vrpn.position` records position and orientation, while `vrpn.velocity`
retains the raw linear and angular vectors. The Adapter combines both projected
streams to project world-frame linear velocity onto the signed body X axis;
that processed scalar is `vrpn.speed`, so lateral slip is excluded.
`command.velocity` separately records the commanded
linear and angular velocity; the Adapter does not consume odometry. `state.power.voltageV`
retains the measured chassis voltage. `state.power.percentage` uses
`PERCENTAGE_STATE_AVAILABLE` for a **voltage-only estimate**, shown with an
approximation sign and explicit label in the UI. The original 24 V / 15 Ah NMC
pack is modeled as 7S, inferred from its chemistry and ~29.2 V full-charge
voltage in the [Scout Mini manual](https://agilexrobotics.gitbook.io/scout_mini/3-shi-yong-yu-kai-fa-getting-started).
The profile's nonlinear interior points sample the 25 C NMC reference-cell OCV
polynomial in [Nejad & Gladwin, Eq. (1)](https://doi.org/10.1109/TIE.2019.2921280)
at 10..90% SOC. The 3.0 V/cell zero and 29.2 V pack full endpoints are heuristic
extensions. This is **not a characterized Scout cell or pack SOC curve**.
At 24.4 V it gives approximately 10%, rather than the old 0% clamp or 45% from
protection-to-full linear interpolation. Load, temperature, imbalance and
aging can bias this approximation; no accuracy bound has been measured.
A three-sample median and 10-second exponential voltage filter damp brief sag
and quantization. Gaps over five seconds reset the filter; zero/non-finite
samples leave percentage unavailable. Raw voltage and chassis low-voltage
alarms remain independent of the filter. No current, IR compensation,
coulomb count or available runtime is inferred from voltage alone.

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

The Mecanum UGV Adapter carries `vrpn.position`, raw `vrpn.velocity`,
processed `vrpn.speed`, and uniquely publishes canonical
`/{namespace}/{pose,twist,accel}`. It also carries `command.velocity`, on-board `state.imu` (`imu`),
`state.power` (`PowerVoltage`, `std_msgs/Float32`, the same topic and type
as Scout), the existing longitudinal/yaw motion-intent operation, and
channel diagnostics. Online state depends on a fresh IMU; projected pose gates
readiness. It has no Scout status or odometry dependency. The scalar speed
is computed in C++ by projecting the world-frame canonical linear vector onto
the body X axis from the canonical pose quaternion, preserving reverse sign
while excluding lateral slip. Motion intent keeps the existing
protobuf contract (there is no lateral field) and maps its three gears to the
deployed SSS Mecanum limits: 0.5/1.0/1.5 m/s longitudinal and approximately
0.5236/1.0472/1.5708 rad/s yaw. Before the first accepted intent, the adapter
publishes no `cmd_vel`; afterward it republishes the latest intent at 10 Hz and
sends a final zero on shutdown. `state.power.voltageV` retains the measured
voltage, while `state.power.percentage` uses
`PERCENTAGE_STATE_UNAVAILABLE`. The physical `mini_mec` mode is not proof of an
exact WheelTec battery SKU, and the profile has no authoritative
voltage-to-SOC curve.

The Mocap Rotor uses two computers and two independent ROS1 lifecycles. Its
existing MAVROS and ROS master remain on the Orin NX; neither the ground Core
nor the ground Adapter starts them. The separately installed onboard Forwarder
subscribes only to six deployment-supplied absolute topics: local pose, local
velocity, IMU, battery, MAVROS state, and MAVROS extended state. Those message
types follow the `external/dev/xgc1` Mocap Rotor reference, but their production
names have no defaults and must come from an actual onboard graph snapshot.

One static Zenoh client session emits only
`xgc2/{robot_id}/up/{local_pose,local_velocity,imu,power,flight_state,forwarder_hb}`.
Each channel has a hard rate and payload bound. GPS, commands, setpoints,
downlink keys, ROS graph bridging, and FS150 process/port reuse are absent. The
ground `xgc_mocap_rotor_ros1_adapter` owns the peer listener, wire validation,
freshness, semantic telemetry, and namespaced ROS1 recovery; it has no MAVROS
message dependency. Unknown battery measurements are represented explicitly
rather than replaced with plausible numbers.

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

Each Adapter Debian owns three generated, immutable installation contracts:

- `/usr/share/xgc2/adapter-definitions/<provider>.json`
- `/usr/share/xgc2/process-definitions/<provider>.json`
- `/usr/share/xgc2/robot-adapter-profiles/<provider>.json`

The install step hashes the final ELF, computes canonical capability and public
Profile contract digests, and validates every message ID/version/fingerprint
against `xgc2-protobuf`. The process definition
accepts only the supervisor-owned `adapterBootstrapFile` parameter and invokes
the executable directly with `--adapter-bootstrap-file` and the complete ROS
Noetic runtime environment. It never relies on a shell or a sourced setup file.

The onboard Forwarder Debian is deliberately different: it owns only
`/usr/share/xgc2/process-definitions/xgc2-mocap-rotor-link.json`. That closed
definition requires the onboard ROS master URI/IP, ground Zenoh endpoint, all
six source-topic mappings, and the pose child frame. Its executable is a stable
`/opt/ros/noetic/lib/...` path and it does not depend on the ground Adapter
Runtime ABI.

Package-local C++ headers are implementation details used only while building
each executable. The Debian packages intentionally export no Catkin header or
library interface.

The binary bootstrap is owner-only mode `0600`. The three servers receive
only their provider, UDS and ROS environment; members arrive through gRPC.
The two Runtime Link providers retain their initial instance specification
and granted capability contracts in the bootstrap.

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

The release path builds and install-checks five independent Adapter Debian
packages plus the independently installable onboard Forwarder package:

```bash
.xgc2/scripts/build_debs_in_docker.sh --output-dir "$PWD/debs"
```

After publishing, the public Focal APT gate installs the exact frozen B2
version in a clean ROS Noetic container and rejects source-tree paths in every
installed runtime manifest:

```bash
.xgc2/scripts/check_public_apt_install.sh
```

Builds require protobuf `0.5.0-20~focal` from source
`99f301ee8725e91ae8149becce92377ea8fbecb0` and client SDK
`0.6.0-17~focal` from source `1aa878d182297778271745543e8b8129507327a7`.
The staged SDK must declare the exact same protobuf dependency. The three type
servers link gRPC and c-ares directly; they do not link the legacy client runtime.
B2 and Mocap Rotor retain their ELF-derived client runtime dependency. Installed
packages omit protobuf and client development dependencies.

## Supervisor launch

The Process Supervisor starts the fixed installed executable directly:

```text
/opt/ros/noetic/lib/xgc_px4_multirotor_ros1_adapter/xgc_px4_multirotor_ros1_adapter_node
/opt/ros/noetic/lib/xgc_scout_mini_ros1_adapter/xgc_scout_mini_ros1_adapter_node
/opt/ros/noetic/lib/xgc_mecanum_ugv_ros1_adapter/xgc_mecanum_ugv_ros1_adapter_node
/opt/ros/noetic/lib/xgc_unitree_b2_ros1_adapter/xgc_unitree_b2_ros1_adapter_node
/opt/ros/noetic/lib/xgc_mocap_rotor_ros1_adapter/xgc_mocap_rotor_ros1_adapter_node
/opt/ros/noetic/lib/xgc_mocap_rotor_zenoh_forwarder/xgc_mocap_rotor_zenoh_forwarder_node
```

For a diagnostic manual launch, pass a real supervisor-generated bootstrap:

```bash
rosrun xgc_px4_multirotor_ros1_adapter \
  xgc_px4_multirotor_ros1_adapter_node \
  --adapter-bootstrap-file /run/xgc2/adapter/processes/<instance>.bootstrap
```

The repository is distributed under the BSD 3-Clause License in `LICENSE`.

### PX4 operation helpers

The PX4 catkin package exports `xgc_px4_multirotor_ros1_adapter_operations` and
`px4_operations.hpp` with pure request builders, response mapping and state
readiness checks. Native service I/O belongs to the type server's shared event
loop. The former service helper and synchronous executor are removed. The
static operations archive links ROS/MAVROS without an Adapter Runtime SDK
runtime dependency.

Private acceptance uses `common/test/test_async_ros_services.py` and
`common/test/test_robot_server.py`; the latter requires an explicitly isolated
ROS test environment and writes startup, registration, forwarding latency,
thread/process, CPU and RSS measurements alongside its log. Never point these
tests at an active station's ROS master.

The source-owned installed-package acceptance entry is
`.xgc2/scripts/test_robot_servers.sh <catkin-build-directory> <empty-results-directory>`.
Run it inside a disposable, network-isolated ROS container after installing
the three packages, with `ROBOT_SERVER_PRIVATE_TEST=1` and
`ROS_MASTER_URI=http://127.0.0.1:11331`. Set
`ROBOT_SERVER_XSIM_BINARY` to the production xsim executable to include its
three-model custom-topic integration, and `ROBOT_SERVER_CORE_TEST_BINARY` to
the compiled Core native-client test to include its 1/20/100 measurements.
The suite exercises actual TCPROS request barriers and cancellation, single
member loss/removal, reconnect, ROS forwarding and vision rates, native command
results, mixed member profiles and slow gRPC consumers. Direct-input pressure
tests use 100 members at 125 Hz pose/twist plus 30 Hz IMU and a separate
single-member burst. Production xsim integration compares actual ROS IMU
fields, covariances and timestamps with semantic telemetry for all three types.
Its JSON measurements report startup/registration, application/library threads,
CPU/RSS and latency.
