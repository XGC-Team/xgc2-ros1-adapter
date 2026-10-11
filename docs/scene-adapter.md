# Scene adapter

`xgc_scene_ros1_adapter` provides the Run-owned `xgc2_scene_adapter` node and the
finite `xgc2-scene-assemble` and `xgc2-scene-prepare` tools. The Debian package is
`ros-noetic-xgc2-scene-adapter`. Executables are installed under
`/opt/ros/noetic/lib/xgc_scene_ros1_adapter/`; its process definition is
`/usr/share/xgc2/process-definitions/xgc2-ros1-scene-adapter.json`.

The authoring core validates `xgc2.scene.v1` documents, resolves scene generators,
assembles sealed elements, manages edits and undo/redo, evaluates motion and saves
YAML atomically. Shared geometry and placement math come from `xgc2_math::scene`
(Math 0.6.0 or newer). The Gazebo lidar fleet owns its sensing schedules, workers
and scan buffers; its edge supplies ROS transport.

## Profiles

Describe declares the selected profile and its explicit capabilities.

| Profile | Capabilities | Simulator application |
|---|---|---|
| `gazebo` | `scene.read`, `scene.edit`, `scene.motion`, `scene.save`; CPU-only lidar facts | Applies scene and motion asynchronously through the Gazebo world host's `/v1/extensions/scene/*` routes. Requires `~simulation_endpoint`; world availability does not gate readiness. GPU lidar backends are rejected. |
| `xsim` | `scene.read`, `scene.edit`, `scene.save` | The simulator uses its scene fixed at startup; live simulator edits report `unsupported`. |
| `physical` | Scene authoring and visualization | Reads, edits and saves documents and publishes visualization. |

## Private ROS parameters

The process definition passes these as standard ROS `_parameter:=value` arguments.

| Parameter | Meaning |
|---|---|
| `~scene_file` | Required scene document or `xgc2.scene-source.v1` source file. |
| `~working_file` | Working copy used for editing and reopening; empty selects no working copy. |
| `~generation_parameters_json` | JSON generator overrides; defaults to `{}`. Python 3 is available for generator commands. |
| `~save_directory` | Directory granted for saves; empty uses the working file's directory, or the source directory when there is no working file. |
| `~target_id` | XRPC target identity and local target fence for world discovery; the definition defaults to `local`. |
| `~xrpc_socket` | Unix socket path allocated by Core in the Run's private runtime directory. |
| `~frozen` | Rejects authoring edits when true; defaults to false. |
| `~profile` | `gazebo`, `xsim` or `physical`; defaults to `gazebo`. |
| `~simulation_endpoint` | World host's absolute Unix socket path from `$session.endpoints.world`, passed as `simulationEndpoint` (`fixedOnly`). The adapter discovers the instance itself. Defaults to empty, meaning no world; `gazebo` rejects empty or relative paths with a named usage error. Other profiles ignore it. |

## Topics and TF

Topic names are relative to the node's ROS namespace, selected by `__ns` (the
definition defaults to `/xgc_scene`).

| Topic | Message type | Publication |
|---|---|---|
| `snapshot` | `xgc2_geometry_msgs/SceneSnapshot` | Latched scene definition. |
| `state` | `xgc2_geometry_msgs/SceneState` | Motion state from a 30 Hz timer. |
| `document` | `std_msgs/String` | Latched envelope JSON text. |
| `markers` | `visualization_msgs/MarkerArray` | Latched visualization. |
| `/tf_static` | `tf2_msgs/TFMessage` | Static obstacle transforms. |
| `/tf` | `tf2_msgs/TFMessage` | Dynamic obstacle transforms. |

The parent frame is the document's `frame`. Obstacle child frames use the prefix
`xgc_scene_<sha256(namespace) first 12 hex>`, followed by epoch, `fixed` or `moving`,
and obstacle identity. The namespace is the resolved ROS namespace, including its
trailing slash.

## Authoring service

`xgc2.scene.authoring` v1 uses XRPC `http.v1` over the Unix socket (XRPC 0.2.0 or
newer):

| Method | Route | Purpose |
|---|---|---|
| GET | `/v1/describe` | Discovery, readiness, profile and capabilities. |
| GET | `/v1/health` | Service health. |
| GET | `/v1/scene` | Current scene envelope. |
| POST | `/v1/commands` | Submit an authoring command. |
| POST | `/v1/commands/result` | Retrieve a command result. |

`GET /v1/describe?wait_ready_ms=N` is unbound discovery: it needs no instance header
and holds until ready or the wait ends, with `N` at most 30000 ms. Other calls bind
to the discovered instance. Command identity comes only from `X-Request-ID`.
The service admits four calls in flight, limits bodies to 8 MiB and responses to
8 MiB plus 65536 bytes, and has a 60 s call timeout.

Both sockets bind before the ROS master exists. While waiting, both Describe
responses have `ready: false` and `facts.ros_master: "waiting"`; held discovery
is released when the adapter is ready. The adapter waits without a startup
deadline, using one bounded XML-RPC liveness call with exponential full jitter
from 250 ms to 10 s. SIGTERM during this wait exits successfully. After binding,
a replaced master ends the process with exit code 3; a master that stops
answering for 5 s ends it with code 4.

Authoring readiness requires its sockets, registered ROS node, loaded scene and
open store. A Gazebo world host is a runtime peer. Edits commit and save locally
while that peer is absent; native application completes asynchronously. The
scene envelope and Describe facts report `synchronized: false`,
`syncRetryable: true` and the consumer's reason until native truth is confirmed.
Command receipts retain the synchronization state at the time of the command.

The simulation client starts its worker when the first scene must be applied.
One held unbound Describe discovers the instance at `~simulation_endpoint`, the
session's world rendezvous path. Reconnect uses full jitter from 250 ms to 10 s and one directory
inotify watch for socket disappearance or appearance. Each peer has one attempt
in flight, at most 15 failed attempts in a rolling minute, and one latest desired
scene; reconnect never holds the authoring or lidar locks. A changed instance
receives the current document, epoch and revision once, followed by its
play/pause/reset state. Native refusals remain visible until an edit or the
existing `resync` command supplies a new revision. There are no periodic scene
replays. Outages log the first failure and at most one suppressed-attempt summary
per 10 s.

The native motion API cannot restore elapsed scene time. If a restarted world
reports a different phase, the adapter keeps the scene unsynchronized and lidar
truth invalid. The existing reset command can return both clocks to zero;
restoring an arbitrary phase requires a world-host API change.

## Lidar fleet

The same process hosts `xgc2.sensor.lidar` v1 (`http.v1`) on a second Unix socket
in every profile. Core waits for unbound Describe on both sockets. Lidar Describe
facts include `target_id`, `profile`, `sensors`, `backend` and `scene_ready`.
Lidar readiness requires its bound control socket and constructed fleet resources;
poses, scene truth and topic subscribers are runtime inputs. A ready fleet can
publish nothing while those inputs are absent, then recover through normal ROS
subscriber callbacks. In `gazebo` the
adapter builds the CPU sampled fleet from the frozen session robot roster.
Default simulator-model lidars remain with the simulator and are excluded from
this fleet. An empty selection creates no engine. Gazebo declares `backend:
"cpu"`; `xsim` and `physical` have no engine, zero sensors and `backend: "none"`.
Their enable control changes no sensor behavior. Sampled selections require
explicit CPU acceleration; GPU requests are refused without fallback.

| Input | Process parameter | Meaning |
|---|---|---|
| `~lidar_xrpc_socket` | `lidarSocketPath` | Required absolute Unix socket path, distinct from `socketPath`; `fixedOnly`. |
| Standard input with `--session-stdin` | `sessionJson` | One frozen session JSON document, at most 8 MiB. Default empty means no fleet. |
| `~lidar_enabled` | `lidarEnabled` | Initial enable state, default true. Launch configuration owns initial state. |
| `~lidar_defaults_json` | `lidarDefaultsJson` | Defaults object, default `{}`. Unknown keys are refused by name. |

Core computes no fleet manifest. The scene workflow copies `$session.runMode`,
`$session.robots` and `$session.visualizationTopics` unchanged into this string
expression, interpolating the JSON arrays inline:

```text
{"runMode":"{{ $session.runMode }}","robots":{{ $session.robots }},"visualizationTopics":{{ $session.visualizationTopics }}}
```

`command.stdinParameter` names `sessionJson`; the command includes
`--session-stdin`. The executable reads stdin only with that flag, before ROS,
threads or sockets start. The supervisor supplies `/dev/null` for absent input,
so the read ends. A present document is validated in every profile. Read errors,
oversized input, invalid JSON or run modes are usage failures naming
`--session-stdin`. The document requires `runMode` (`simulation`, `hybrid` or
`physical`), `robots` (an array of frozen RobotFact objects) and
`visualizationTopics` (the unchanged layer arrays). The contract is in
[roster.hpp](../src/scene_ros1_adapter/include/xgc_scene/lidar/roster.hpp) and its
[legacy golden tests](../src/scene_ros1_adapter/test/lidar/roster_test.cpp).

The adapter invokes `buildFleet` only in `gazebo`. It follows
`binding.simulationSensors.simpleLidar` and, for hybrid runs,
`binding.hybridSource`. Physical runs select no fleet. A sampled sensor is
selected by penetrating/depth mode, requested beams, or the legacy frozen
preset selection; a boolean/default-model lidar does not become a fleet sensor.
The builder keeps the legacy numeric overrides and selection refusals. It
normalizes namespaces and requires explicit `acceleration: "cpu"`; penetrating
with beams is refused. The input roster is limited to 512 members before
filtering. A visible semantic PointCloud2 declaration for
`/xgc/scene/reference_cloud` adds the reference publisher, including for a
reference-only fleet. The internal manifest is schemaVersion 1, at most 1 MiB;
it is built inside the adapter and is not a process input.

Node defaults passed to the builder/parser are `range`, `min_range`,
`h_fov_deg`, `v_fov_deg`, `h_res`, `v_res`, `surface_spacing`, `vertical_slab_tan`,
`fx`, `fy`, `cx`, `cy`, `noise_std` and `seed`. Engine defaults are `yaw_only`,
`pose_timeout`, `worker_threads`, `vehicle_bodies` and `vehicle_body_radius`.
Values emitted from the roster take precedence. The internal manifest owns
mode/crop choices, rates and beams; camera width/height derive from hRes/vRes.
Ineffective node defaults for those choices are not accepted.

Sensor pose topics default to `/vrpn_client_node<namespace>/pose`. At the fleet
seam, a body roster selects body truth for nonpenetrating sensors; penetrating
sensors retain the default source, and explicit `poseTopic` wins in either case.
These are the legacy fleet resolution rules. The session builder derives no
body roster or sensor settings from asset/profile facts.

| Topic | Contract |
|---|---|
| Resolved sensor and body pose topics | `geometry_msgs/PoseStamped`, frame `world`; Math validates poses and observation age. Invalid truth suppresses scans. |
| `<namespace>/simple_lidar/points` | `sensor_msgs/PointCloud2`, float32 `x y z`, point_step 12. |
| `<namespace>/simple_lidar/beams` | Advertised only when requested for raycast/depth sensors: float32 `x y z dx dy dz range hit`, point_step 32. |
| `/xgc/scene/reference_cloud` | Full scene exterior XYZ reference; queue 1, latched. Engine Publish/Clear actions replace the latch. |

Sensor clouds have frame `world`, height 1, width equal to the record count,
`is_dense: true`, pose timestamps, queue 1 and no latch. Each output message is
reused. Subscriber connect/disconnect callbacks set demand; an undemanded sensor
does no scan work and consumes no noise samples. Reference demand also follows
subscriber callbacks. One ROS timer on lane 0 drives the fleet at the smallest
sensor period, capped at 1 kHz; a reference-only fleet uses 1 Hz. ROS simulated
time drives that timer. The fleet engine owns schedules, its worker pool and
scan buffers.

Scene truth is tapped inside `SceneNode` at the native application boundary:
pending apply and `gazeboFailure` invalidate immediately; a matching native report
admits the matching `applied_snapshot_` and state. `applicationView` gates the
tap against the native report. Math converts `SceneObstacleDescription` to
obstacles. New documents/epochs, refused geometry, non-world frames and
unsynchronized simulation clear the reference and suppress scans. Dynamic
geometry is installed only when its matching state changes. No scene truth ROS
subscriptions or legacy Gazebo obstacle messages are used.

`POST /v1/configure` on the lidar socket retains the existing source control wire:
`expected_revision`, `configuration: {"enabled": false|true}` and
`operation_timeout_ms`, with `X-Request-ID` and the discovered instance header.
Status/health and operation receipt/wait routes are unchanged. Control applies
on lane 0. While disabled, the fleet scans and publishes nothing; pending
reference clears wait for re-enable. Control stops before the timer and worker
pool, and ROS handles remain alive until workers finish.

The old node's preset YAML files and node-level preset parameters are not
carried over. The fleet follows the frozen roster through the adapter's legacy
selection builder. Topic patterns (`pose_topic_pattern`, `output_topic_pattern`,
`beams_topic_pattern`, `vehicle_ids`, `enabled_vehicles`, `num_vehicles`) and file
manifest provisioning are not carried over. Core passes session facts unchanged;
the adapter constructs the fleet, including the legacy selection behavior that
formerly lived in `prepare_fleet.py`.

## Finite CLI usage

Assembly consumes JSON on stdin and writes JSON on stdout:

```sh
rosrun xgc_scene_ros1_adapter xgc2-scene-assemble < assembly.json > assembled.json
```

The input contains `request`, `documents` and `extrinsic`: the request selects
2 to 64 sealed elements and their placements, documents supplies their YAML,
and extrinsic supplies the camera transform. The output contains `sceneDocument`
(YAML text) and the selected `task` identifier. Invalid input exits 2 with one error
line on stderr.

Preparation consumes the scene context through an argument and writes JSON on stdout:

```sh
rosrun xgc_scene_ros1_adapter xgc2-scene-prepare \
  --context-json "$(cat context.json)" \
  --resource-root /path/to/resources --workspace-root /path/to/workspace
```

The roots can also come from `XGC_USER_FILES_DIR` and `XGC_JOB_ARTIFACT_ROOT`.
To materialize a Gazebo world, add `--prepare-world --socket-path PATH --target-id ID
--configuration-revision INTEGER`. The context carries the actual `openingRunId`,
and the workspace root is an existing directory granted by the provider. Preparation
writes only inside that grant. `xgc2-scene-prepare --help` lists the arguments.

## Tests

The golden corpus in `test/scene_golden` is the behavioral reference. Core sources
and replay tests are globbed by the package CMake. The catkin CI job builds the
node and both CLIs, runs the catkin tests and the package's CTest suite, packages
the installed files, and checks the scene package's ownership and library resolution.
