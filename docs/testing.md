# Tests

Four layers, from the cheapest to the one closest to a Run.

## 1. Unit tests (gtest, no ROS master)

`catkin_make run_tests` builds and runs them; each is also an executable in
`devel/lib/<package>/`.

| Executable | What it covers |
|---|---|
| `xgc_ros1_adapter_common_server_test` | The server core against a fake `RosBinding` and fake robots, over real gRPC on a Unix socket: Describe held until ready, ApplyMembers for several robots with failures per robot, idempotent re-apply and conflicts, the status stream and removal tombstones, Execute per robot in order, an uncertain result latching a robot, the drain in order (in-flight command finished or cancelled, final stop of every robot, one flush before the robots are destroyed, ROS shutdown last), SIGTERM, refusal of new work while draining, master replaced or unreachable or briefly away, ROS shutting the node down, and a master that never answers at start. |
| `xgc_ros1_adapter_common_options_test` | The command line: required flags, identifiers, retired flags are refused. |
| `xgc_ros1_adapter_common_domain_test` | Helpers shared by the robots: the localization projection and its Run-scoped parameters, the positioning liveness window, the robot message encoder. |
| `xgc_ugv_ros1_adapter_test` | The UGV runtime for both variants: every behavior the two chassis share is run for each, and what differs (limits, lateral motion, health source, battery filter, stream-rate window) is checked per variant. |
| `xgc_px4_multirotor_ros1_adapter_test`, `..._operations_test` | The PX4 runtime and its arm, mode, reboot and force-disarm interpretation. |
| `xgc_ros1_tools_adapter_unit_tests` | The Tools codec, registry and invoker. |
| Scene core tests and `*_replay_test` | Document and YAML validation, motion, generation, assembly, preparation and store behavior against `test/scene_golden`. CI runs them with CTest in `build/scene_ros1_adapter` after the catkin test targets. |

## 2. Generator and definition tests (Python)

```
python3 -m unittest discover -s test -p 'test_*.py'
```

- `test_generate_contract_metadata.py`: the contract tables generated from the profiles
  are complete and consistent with the protobuf registry, compile as C++17 and answer the
  lookups the runtimes make.
- `test_profile_catalog.py`: the catalog Core reads is exactly what the profiles produce.
- `test_process_definitions.py`: every shipped process definition matches the executable it
  starts (flags, providers, services on the socket parameter, stop and readiness fields).

`src/common/test/test_async_ros_services.py` runs the TCPROS service client against a
controlled transport fixture (`ASYNC_ROS_SERVICES_DRIVER=<async_ros_services_driver>`).

## 3. Integration tests (private ROS master)

`test/integration` runs the executables against a `roscore` of their own and drives them over
their sockets the way a Run does. Each test uses a random master port, a private directory
for sockets and logs, and a private `ROS_HOME`; nothing is shared with other masters. They need
a sourced ROS environment and

```
XGC_TEST_DEVEL=<catkin devel space>   # executables, and robot_server_client
XGC_TEST_BUILD=<catkin build space>   # generated profile catalogs
XGC_TEST_REGISTRY=<registry.json of xgc2-protobuf>
```

(`robot_server_client` and `async_ros_services_driver` are built on demand:
`make robot_server_client async_ros_services_driver` in the catkin build space.)
`XGC_TEST_INSTALL_ROOT=/` tests the installed packages instead: their executables and installed
catalogs. `.xgc2/scripts/test_robot_servers.sh` does that in the release container.

```
python3 -m unittest discover -v -s test/integration -p 'test_*.py'
```

Fake vehicles (`fake_robots.py`) publish the topics a profile reads, answer the services it
calls and record what the server publishes to the vehicle; the robot's own ROS interface is
derived from its profile (`harness.Robot`), so a change of a profile changes what is faked.
`robot_server_client` is a small gRPC client (Python has no gRPC on the build machines): one RPC
per invocation, JSON in and out.

| File | Cases |
|---|---|
| `test_ugv_servers.py`, run for Scout Mini and Mecanum | Describe with the facts and a bounded wait; one ApplyMembers for robots of three profiles; the status stream; failures per robot, an identical re-apply, a configuration and an epoch conflict; the motion intent reaching the vehicle at the limits of the chassis and the release; removal of a robot (final zero, subscriptions gone, tombstone); Shutdown and SIGTERM with the final zero and the process gone from the ROS graph; a server that stops at once; three consecutive Runs with different robot lists, stopped by Shutdown, SIGTERM and Shutdown, checking the PIDs, `rosnode list` and the master's registrations; the thread count at 1, 20 and 100 robots; two robot types on one master. |
| `test_ugv_servers.py` (`MasterLossTest`) | The master restarted (exit 3), gone (exit 4), briefly stopped and back (the server keeps serving), silent (calls to it end within their budget, exit 4), never there at start (exit 4), `rosnode kill` (exit 5), a second node of the same name displacing the first (exit 5, the newer server keeps serving). |
| `test_px4_server.py` | The same lifecycle for PX4 against fake MAVROS: robots of three profiles, arm and flight mode through the vehicle's services, a mode the profile does not allow never reaching the vehicle, the setpoint stream at the gear limit and its final zero setpoint at Shutdown, SIGTERM and removal, a command in flight during the drain (finished within the budget, or cancelled and reported uncertain), consecutive Runs, a restarted master. |
| `test_tools_server.py` | The Tools process: discovery and the held Describe with the master facts, a publish reaching a subscriber, a service call reaching a provider, consecutive Runs leaving no ROS registration behind, a restarted master (exit 3), a master that stays away or is never there (exit 4), `rosnode kill` (exit 5). |

Not covered: the real vehicles' drivers (Scout SDK, MAVROS and PX4), Gazebo and xsim worlds,
and the Core and workflow side of a Run. A Scout Mini, Mecanum or PX4 vehicle is stood in for
by topics and services of the right types; their behavior is the one the profiles describe.

## 4. Release container

`.xgc2/scripts/build_debs_in_docker.sh` runs layers 1 and 2 on the checkout, builds with the
release compiler, installs the packages and runs `check_installed_packages.sh` (files,
dependencies, library resolution, process definitions and catalogs against the profiles) and
then `test_robot_servers.sh` (the transport test and layer 3 against the installed packages).
The build also checks the `xgc2_scene_adapter`, `xgc2-scene-assemble` and
`xgc2-scene-prepare` targets and runs the Scene package's CTest suite. Its installed
package gate checks all three executables, the process definition, dependency floors
and `xgc2-scene-prepare --help`. Third-party build dependencies come from the managed
build image; published Math and scene messages are first-party inputs.
