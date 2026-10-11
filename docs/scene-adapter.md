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
| `gazebo` | `scene.read`, `scene.edit`, `scene.motion`, `scene.save`; CPU-only lidar facts | Applies scene and motion through the Gazebo world host's `/v1/extensions/scene/*` routes. Requires `~simulation_service_ref_json`. GPU lidar backends are rejected. |
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
| `~target_id` | XRPC target identity; the definition defaults to `local`. |
| `~xrpc_socket` | Unix socket path allocated by Core in the Run's private runtime directory. |
| `~frozen` | Rejects authoring edits when true; defaults to false. |
| `~profile` | `gazebo`, `xsim` or `physical`; defaults to `gazebo`. |
| `~simulation_service_ref_json` | JSON service reference of the Gazebo world host, required for `gazebo`. |

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

## Lidar fleet

The same process hosts `xgc2.sensor.lidar` v1 (`http.v1`) on a second Unix socket
in every profile. Core waits for unbound Describe on both sockets. Lidar Describe
facts include `target_id`, `profile`, `sensors` and `backend`. The `gazebo` profile
uses the CPU fleet when the manifest contains sensors or a reference cloud and
declares `backend: "cpu"`. An empty manifest creates no engine. In `xsim` and
`physical` there is no engine, the sensor count is zero, `backend` is `"none"`,
and the enable control changes no sensor behavior. GPU requests are refused;
there is no fallback backend.

| Private ROS parameter | Process parameter | Meaning |
|---|---|---|
| `~lidar_xrpc_socket` | `lidarSocketPath` | Required absolute Unix socket path, distinct from `socketPath`; `fixedOnly`. |
| `~lidar_fleet_json` | `lidarFleetJson` | Fleet manifest text, schemaVersion 1, at most 1 MiB. Default empty creates no fleet. |
| `~lidar_enabled` | `lidarEnabled` | Initial enable state, default true. Launch configuration owns initial state. |
| `~lidar_defaults_json` | `lidarDefaultsJson` | Defaults object, default `{}`. Unknown keys are refused by name. |

The exact defaults keys consumed by the manifest parser are `range`, `min_range`,
`h_fov_deg`, `v_fov_deg`, `h_res`, `v_res`, `surface_spacing`, `vertical_slab_tan`,
`fx`, `fy`, `cx`, `cy`, `noise_std` and `seed`. Engine defaults are `yaw_only`,
`pose_timeout`, `worker_threads`, `vehicle_bodies` and `vehicle_body_radius`.
The manifest owns `mode`, `keepBuried`, `headingCrop`, `headingCosMin`, rates and
beam publication; camera width/height are derived from `hRes`/`vRes`.
Those manifest-owned choices are not accepted as ineffective node defaults.

The schema is defined by [parseManifest](../src/scene_ros1_adapter/include/xgc_scene/lidar/manifest.hpp)
and exercised by [manifest tests](../src/scene_ros1_adapter/test/lidar/manifest_test.cpp).
For example:

```json
{"schemaVersion":1,"robots":[{"namespace":"/A","poseTopic":"/A/pose",
 "mode":"raycast","rateHz":10,"hRes":90,"vRes":8,"publishBeams":true}],
 "referenceCloud":{"surfaceSpacing":0.1}}
```

`robots` contains 0 to 512 sensors, with an absolute namespace and an optional
pose topic. Without an explicit topic it uses `/vrpn_client_node<namespace>/pose`.
An optional `bodies` roster supplies stable IDs, namespaces, truth pose topics and
sphere geometry (`radiusMeters`); nonpenetrating sensors use their body's truth
topic by default, exclude their own body, and retain unsensed bodies. Penetrating
sensors retain their original pose source unless overridden. A reference-only
manifest has an empty `robots` array and creates no robot sensor topics.

| Topic | Contract |
|---|---|
| Manifest sensor and body pose topics | `geometry_msgs/PoseStamped`, frame `world`; Math validates poses and observation age. Invalid truth suppresses scans. |
| `<namespace>/world_lidar/points` | `sensor_msgs/PointCloud2`, float32 `x y z`, point_step 12. |
| `<namespace>/world_lidar/beams` | When requested for raycast/depth sensors: float32 `x y z dx dy dz range hit`, point_step 32. |
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
pending apply and `gazeboFailure` invalidate immediately; a completed apply
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

Presets under `config/presets` are retired here: Core resolves presets to values.
The manifest replaces `pose_topic_pattern`, `output_topic_pattern`,
`beams_topic_pattern`, `vehicle_ids`, `enabled_vehicles` and `num_vehicles`.
The file variant of the fleet parameter is dropped; Core passes text. The
manifest producer, formerly `prepare_fleet.py` run by the retired provisioning
script, is Core's workflow concern and is not part of this repository.

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
