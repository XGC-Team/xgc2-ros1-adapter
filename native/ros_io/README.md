# Native ROS1 edge checks

This owner adapts physical ROS1 telemetry and command topics to the existing
module records. The process ROS initialization and clock/output guard remain
in `ros_edge`, shared with `ros_clock_source`; no second guard is introduced.
The native edge retains the existing sensor, MAVROS command, reference,
planar-PVA and estimator projections. `attitude_target_full` preserves every
MAVROS mask bit and payload field. The historical `sim_hover_thrust` port is
the controller's native hover-thrust-estimator output and keeps its existing
ROS topic and trace configuration.

The adapter no longer owns an embedded simulator, simulated FCU facade,
provider ROS service, simulated sensor publishers, or plant/public simulation
headers. The old simulator-only helper tests and per-body Host live drivers
retired with that implementation. The standalone `xsim` product owns its
original FS150/Scout/Mecanum model and native management API tests.
Retired simulation config keys are rejected rather than ignored.

Run the original local arithmetic and full-attitude wire checks:

```bash
bash native/ros_io/run-slice-test.sh
bash native/ros_io/run-attitude-target-test.sh
```

`run-attitude-target-test.sh` requires the installed Robotics Interfaces
headers and real MAVROS message headers. Use the existing prefix environment
variables when the owning headers are in a private test prefix.

The original clock test builds/loads the real native edge and uses its own
private ROS master. It checks publisher identity, reset/backward-time gates,
queue-overflow behavior, suppressed backlog, and normal command/setpoint
recovery. It also checks that retired simulation ports/config are unavailable:

```bash
bash native/ros_io/run-clock-test.sh
```

The normal package CI runs these three checks against its exact source-built
DSO before packaging, alongside the original catkin robot Adapter tests.
The Edge library, `ros_slice.hpp`, and their Helpers CMake exports remain
owned and packaged here. The retired `sim_odometry.hpp` export and both old
simulator Debian dependencies are absent from the new native-bridge payload.
