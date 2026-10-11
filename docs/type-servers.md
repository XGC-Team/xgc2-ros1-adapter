# Robot type servers

One process per robot type and ROS master, started by the experiment workflow of a Run
and ended with it. It serves every robot of that type in the Run over one gRPC service on a
Unix socket (`xgc.robot.v1.RobotAdapterServerService`, XRPC grpc.v1, service name
`xgc2.robot-adapter`). No server outlives its Run and none keeps state from one Run to the
next: a Run that starts again starts a new process, registers its robots again and gets fresh
ROS registrations.

## Lifecycle

1. The Run starts the executable from its process definition (see
   [process-definitions.md](process-definitions.md)). The process creates the socket in the
   private runtime directory of the Run, initializes roscpp and starts its workers. It may
   start before the Run's ROS master exists and waits for that master without a startup
   deadline, then registers its node.
2. **Ready** is reported by `Describe`. Until the node is registered with the master and the
   workers run, `Describe` with `wait_ready_ms` is held (see below) and every other
   operation is refused with `FAILED_PRECONDITION`.
3. The workflow sends **one `ApplyMembers` with all robots of this type** in the Run.
   Registering a robot creates its ROS subscriptions and publishers under its namespace.
   Robot topics need not exist yet: registration is accepted and health reports missing
   telemetry until publishers appear. Subscriptions reconnect when publishers restart.
   Failures are reported per robot; the other robots are unaffected.
4. During the Run the workflow reads telemetry with one `SubscribeStatus` stream and sends
   robot operations with `Execute`.
5. The Run ends the server with `RemoveMembers` (optional), then `Shutdown`, then SIGTERM,
   then SIGKILL after the grace of the process definition. `Shutdown` and SIGTERM do the same
   drain (below) and the process exits 0.

The server uses the ROS master URI supplied at process start (`--ros-master-uri`, `--ros-ip`)
and waits for its first binding. While waiting it serves `Describe` with `ready:false` and
`facts_json.reason: "waiting for ROS master binding"`; SIGTERM/SIGINT exits 0. It never
re-binds after startup. It watches the identity of the bound master once per second
(`getPid`, `getUri` and `/run_id`); if the master is replaced (another pid or a changed known
run id, even on the same URI) or stays unreachable for 5 s, it stops its robots like a drain would, and
exits with a status that names the reason. The Run that owns it fails or restarts it
explicitly.

### Command line

```
--socket-path PATH   Unix socket to create (its directory exists and is private)
--run-id ID          the owning Run; every member must carry it
--node-name NAME     ROS node name; must include the Run id (checked) so that two
                     Runs on one master never share a name and displace each other
--provider TYPE      robot type: xgc2-px4-multirotor-ros1-adapter,
                     xgc2-scout-mini-ros1-adapter or xgc2-mecanum-ugv-ros1-adapter
--target-id ID       XRPC target the service is published on
--ros-master-uri URI the Run's ROS master
[--ros-ip ADDRESS]   ROS_IP of this host
```

Characters of `--node-name` that are not legal in a ROS name become `_`, and the Run id is
compared with the same replacement. A name without the Run id is an invalid command line
(exit 2).

### Exit status

| Status | Meaning |
|---|---|
| 0 | stopped by `Shutdown` or SIGTERM/SIGINT, drained |
| 1 | startup or internal failure (the log names it) |
| 2 | invalid command line |
| 3 | the ROS master was replaced |
| 4 | the bound ROS master stayed unreachable for 5 s |
| 5 | ROS shut the node down (`rosnode kill`) |

On 3, 4 and 5 the server stops its robots first, with the drain below.

## RPC contract

`Describe` is the discovery call and needs no instance. Every other call is bound to the
instance id that `Describe` returned (`service_ref.instance_id`), as XRPC requires; the
server rejects a call for another instance, so a client never talks to a process that took
the socket over after a restart.

### Describe

`Describe(wait_ready_ms)` answers `DescribeResponse`:

- `service_ref`: service `xgc2.robot-adapter`, profile `grpc.v1`, the socket as endpoint, the
  target id and the **instance id**, new for every process start.
- `provider_definition_id`, and the limits `maximum_robots` (256), `maximum_members` (256),
  `maximum_batch_members` (256) and `maximum_commands_per_robot` (32).
- `ready`: true once the node is registered with the master and the server accepts work; false
  while starting, draining or stopped. Robot topics and telemetry never gate readiness.
- `facts_json`: the domain dependencies that make the service valid:

  | Fact | Value |
  |---|---|
  | `ros_master_uri` | the master the process was started with |
  | `ros_master_pid`, `ros_master_reported_uri` | identity of that master as it reported itself |
  | `ros_master_run_id` | the `/run_id` parameter of the master (roslaunch/roscore), `""` for a bare `rosmaster` |
  | `ros_node` | the node name in the ROS graph |
  | `run_id`, `provider` | as started |
  | `members` | robots registered |
  | `state` | `starting`, `ready`, `draining` or `stopped` |
  | `reason` | `waiting for ROS master binding` while waiting for startup binding, otherwise `""` |

`wait_ready_ms` (0 to 30000; more is `INVALID_ARGUMENT`) holds the call without polling until
`ready` or the wait ends, and answers 250 ms before the caller's own deadline at the latest so
the caller gets the current state. At most four waits are held at once.

### ApplyMembers

`ApplyMembers(members[])` registers up to 256 robots in one call. A `MemberRegistration` is the
member identity (`target_id`, `run_id`, `robot_id`, `connection_epoch`) and the
`RobotResource` (`profile_id`, `profile_digest`, `parameters`, enabled `channels`). The answer
has one entry per robot, in order, with an `AdapterError` when that robot failed:

| Code | Cause |
|---|---|
| `member-registration-failed` | invalid identity, other target, **other Run**, unknown profile or digest mismatch, missing namespace or parameter, a ROS environment other than the server's, the ROS registration failing or ending the budget (3 s per robot) |
| `namespace-conflict` | another robot of the Run uses the namespace |
| `member-configuration-conflict` | the robot is registered with another configuration |
| `member-epoch-conflict` | the robot is registered with another `connection_epoch` |

Applying a registration that is already in place changes nothing. A new configuration for
a robot needs `RemoveMembers` first. `connection_epoch` must not be 0.

### RemoveMembers

`RemoveMembers(members[])` removes robots. Removing a robot cancels its queued commands,
stops its native object (which publishes the final zero command if a motion intent is
active), flushes what it published, and releases its ROS subscriptions and publishers. It is
idempotent for robots that are not registered; a wrong `connection_epoch` is
`member-epoch-conflict`. Allowed while draining.

### Execute

`Execute(identity, operation)` runs a robot operation and answers with its `OperationEvent`.
The operation names the robot's operation by `context.endpoint_id` (the `operation_id` of the
profile channel) and carries the typed input payload.

- The robot is found by `robot_id`, `run_id` and `connection_epoch`; otherwise the event is
  `member-unavailable`.
- Commands of one robot run one at a time in arrival order, at most 32 queued
  (`robot-queue-full`). A command whose deadline has passed is not run (`deadline-exceeded`).
- A command whose outcome is not known (`OPERATION_PHASE_UNCERTAIN`) latches the robot: the
  following commands are rejected with `previous-result-unknown` until the robot is removed
  and registered again. A command is never repeated by the server.
- While the server drains, new commands are rejected with `server-draining`.

### SubscribeStatus

One stream of `SubscribeStatusResponse` frames, at most four streams. The first frame carries
the latest message of every channel of every robot; later frames carry what changed, as
`MemberStatus {identity, messages[]}`. A removed robot is announced once with `removed: true`.
The stream ends when the server stops, after the subscribers have been given the removals.

### Health

`Health` answers `serving` (true only while ready), the instance id and the number of robots.

### Shutdown

`Shutdown` starts the drain and answers `serving: false` when it has finished, just before the
process exits. If the caller's deadline ends first the answer is `DEADLINE_EXCEEDED`; the drain
continues and the process still exits.

## Drain

`Shutdown`, SIGTERM and SIGINT (and a lost master) run the same sequence:

1. Refuse new members and commands.
2. Wait up to 2 s for commands in flight to finish; cancel the rest. A cancelled command
   that had already reached the vehicle ends as `UNCERTAIN`.
3. Stop every robot: cancel queued commands, publish the **final zero command** of every robot
   that was moving, release the callbacks.
4. Flush publications: wait until the final commands have left the process. roscpp sends a
   message from a transport thread, and a publisher destroyed before that would drop it.
5. Destroy the robots (their ROS subscriptions and publishers), announce the removals on the
   status streams (up to 500 ms), answer `Shutdown`.
6. Stop the workers, finish the RPCs (up to 5 s), unregister the node (`ros::shutdown`), exit.

Registrations the server made are gone from the master when the process has exited: the node
and every topic and service it held.

## Master calls

roscpp waits for a master that accepts connections but never answers. The server replaces
`ros::master::execute` (in `libxgc2_robot_server_ros1`, ahead of libroscpp in the link order) with a
bounded one: 3 s per call, and one budget per lifecycle step (registration of a robot, removal,
drain). After binding, a silent master fails a member registration within its budget (3 s).
The master watch reports it unreachable after 5 s of failed probes, and the server stops its robots and
exits 4. In the integration test a master that stops answering ends the server in 15 s.

## Threads

The thread count does not depend on the number of robots. Besides the gRPC threads and the
roscpp threads the server has: main, `robot-members` (registration and removal),
`robot-commands` (command scheduler and the TCPROS service client), two `robot-callback` lanes
that run the robots' ROS callbacks, and `master-watch`. A robot is data plus ROS handles; it
owns no thread. The integration tests count 15 to 16 threads at 1, 20 and 100 robots.

## Code

- `src/common/include/xgc2_ros1_robot_adapter/robot_server.hpp`: `RobotServer`,
  `RosBinding`, `NativeRobot`, `ServerOptions`, exit status.
- `src/common/src/robot_server.cpp`: the server. `ros1_binding.cpp`, `ros1_run.cpp`,
  `bounded_ros_master.cpp`, `async_ros_services.cpp`: the ROS1 binding.
- A type adds a `RobotFactory` and its generated contract: `src/px4_multirotor_ros1_adapter`,
  `src/ugv_ros1_adapter`. The contract (profile channels, parameters, message ids) is generated
  from `profiles/ros1/*.yaml` by `tools/generate_contract_metadata.py`; the profile catalog
  the Core reads is generated by `tools/generate_profile_catalog.py` and installed as
  `/usr/share/xgc2/robot-adapter-profiles/<provider>.json`.
