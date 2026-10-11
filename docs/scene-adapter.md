# Scene adapter

`xgc_scene_ros1_adapter` provides the Run-owned `xgc2_scene_adapter` node and the
finite `xgc2-scene-assemble` and `xgc2-scene-prepare` tools. The Debian package is
`ros-noetic-xgc2-scene-adapter`. Executables are installed under
`/opt/ros/noetic/lib/xgc_scene_ros1_adapter/`; its process definition is
`/usr/share/xgc2/process-definitions/xgc2-ros1-scene-adapter.json`.

The authoring core validates `xgc2.scene.v1` documents, resolves scene generators,
assembles sealed elements, manages edits and undo/redo, evaluates motion and saves
YAML atomically. Shared geometry and placement math come from `xgc2_math::scene`
(Math 0.6.0 or newer). Simulators own sensing, scheduling and buffers.

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
