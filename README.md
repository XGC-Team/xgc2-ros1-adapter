# XGC2 ROS1 Robot Adapters

This Catkin workspace contains two robot-domain Adapter Runtime applications.
An Adapter is a general capability plugin for Core or Agent; these two products
specialize that abstraction for PX4 multirotors and Scout Mini robots.

| ROS package | Debian package | Provider definition | Robot profile |
| --- | --- | --- | --- |
| `xgc_px4_multirotor_ros1_adapter` | `ros-noetic-xgc2-px4-multirotor-adapter` | `xgc2-px4-multirotor-ros1-adapter` | `px4.multirotor.ros1.v5` |
| `xgc_scout_mini_ros1_adapter` | `ros-noetic-xgc2-scout-mini-adapter` | `xgc2-scout-mini-ros1-adapter` | `scout-mini.ros1.v3` |

The generic C++ Adapter Runtime SDK owns registration, trusted bootstrap,
session fencing, capability dispatch, flow control, reconnects, and terminal
result delivery. This repository owns robot semantics, ROS1-native mappings,
profile contracts, and native safety policy.

## Runtime contract

Core resolves each provider from the installed robot profile catalog. A
provider instance uses a `robot-group` scope containing `target-id`, `run-id`,
and `provider`; each invocation and telemetry source uses a separate
`robot-resource` subject containing `target-id`, `run-id`, and `robot-id`.

Both products expose:

- `xgc.robot.telemetry@1`: `telemetry` source of serialized
  `xgc.robot.v1.RobotMessage`

The PX4 product also exposes `xgc.robot.command@1`:

- `arm` with `xgc.semantic.aerial.v1.ArmRequest`
- `set-flight-mode` with `xgc.semantic.aerial.v1.ModeRequest`
- `reboot-autopilot` with `xgc.semantic.aerial.v1.AutopilotRebootRequest`

PX4 command operations require deadlines and idempotency keys. A successful
operation returns the registry-owned `xgc.v1.Empty` payload. Native ROS service
calls are not advertised as cancellable after dispatch.

The installed Profile v3 catalog owns each operation's closed JSON parameter
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

Scout Mini telemetry consumes `odom`, `imu/data_raw`, and `scout_status` under
the configured namespace. The Scout profile intentionally exposes no command
capability.

High-bandwidth images, point clouds, and TF visualization remain on their
native ROS visualization paths rather than the semantic telemetry source.

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
  ros-noetic-mavros-msgs \
  ros-noetic-nav-msgs \
  ros-noetic-scout-msgs \
  python3-jsonschema \
  python3-yaml

python3 -m unittest discover -v -s test -p 'test_*.py'

source /opt/ros/noetic/setup.bash
catkin_make
catkin_make run_tests
catkin_test_results --verbose build/test_results
```

The release path builds and install-checks both independent Debian packages:

```bash
.xgc2/scripts/build_debs_in_docker.sh --output-dir "$PWD/debs"
```

## Supervisor launch

The Process Supervisor starts the fixed installed executable directly:

```text
/opt/ros/noetic/lib/xgc_px4_multirotor_ros1_adapter/xgc_px4_multirotor_ros1_adapter_node
/opt/ros/noetic/lib/xgc_scout_mini_ros1_adapter/xgc_scout_mini_ros1_adapter_node
```

For a diagnostic manual launch, pass a real supervisor-generated bootstrap:

```bash
rosrun xgc_px4_multirotor_ros1_adapter \
  xgc_px4_multirotor_ros1_adapter_node \
  --adapter-bootstrap-file /run/xgc2/adapter/processes/<instance>.bootstrap
```

The repository is distributed under the BSD 3-Clause License in `LICENSE`.
