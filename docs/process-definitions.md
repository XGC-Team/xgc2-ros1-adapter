# Process definitions

Each package installs the definitions of the processes it provides into
`/usr/share/xgc2/process-definitions/`. Core loads every `*.json` there next to its own
platform definitions. The files are written by hand next to the code they start
(`src/<package>/process/`); nothing generates them.

| File | Package | Process |
|---|---|---|
| `xgc2-px4-multirotor-ros1-adapter.json` | `ros-noetic-xgc2-px4-multirotor-adapter` | PX4 multirotor type server |
| `xgc2-scout-mini-ros1-adapter.json` | `ros-noetic-xgc2-ugv-adapter` | Scout Mini type server |
| `xgc2-mecanum-ugv-ros1-adapter.json` | `ros-noetic-xgc2-ugv-adapter` | Mecanum UGV type server |
| `xgc2-ros1-tools-adapter.json` | `ros-noetic-xgc2-ros1-tools-adapter` | ROS1 Tools |
| `xgc2-ros1-scene-adapter.json` | `ros-noetic-xgc2-scene-adapter` | ROS1 Scene |

The robot profile catalogs the workflow sends with `ApplyMembers` are installed next to them
in `/usr/share/xgc2/robot-adapter-profiles/<provider>.json` (see
`tools/generate_profile_catalog.py`).

## Shape

The schema id is `xgc.execution.process/v1`. A definition is a direct executable with typed
parameters and optional stdin; there is no wrapper script, probe command or environment preparation.

```json
{
  "apiVersion": "xgc.execution.process/v1",
  "definitions": [{
    "id": "xgc2-scout-mini-ros1-adapter",
    "version": "0.7.0",
    "label": "Scout Mini ROS1 type server",
    "parameters": {
      "properties": {
        "socketPath":   {"type": "string", "fixedOnly": true},
        "runId":        {"type": "string", "fixedOnly": true},
        "targetId":     {"type": "string", "default": "local", "fixedOnly": true},
        "rosMasterUri": {"type": "string", "default": "http://127.0.0.1:11311"},
        "rosIp":        {"type": "string", "default": ""}
      },
      "required": ["socketPath", "runId", "targetId", "rosMasterUri"]
    },
    "command": {
      "executable": "/opt/ros/noetic/lib/xgc_ugv_ros1_adapter/xgc_ugv_ros1_adapter_node",
      "args": ["--socket-path", "${socketPath}", "--run-id", "${runId}",
               "--node-name", "xgc_scout_mini_${runId}",
               "--provider", "xgc2-scout-mini-ros1-adapter", "--target-id", "${targetId}",
               "--ros-master-uri", "${rosMasterUri}", "--ros-ip", "${rosIp}"]
    },
    "services": [{"service": "xgc2.robot-adapter", "profile": "grpc.v1",
                  "endpointParameter": "socketPath", "api_version": "v1"}],
    "readiness": {"kind": "describe", "startGraceMs": 15000, "timeoutMs": 30000},
    "stop": {"graceMs": 25000, "rpc": "Shutdown"}
  }]
}
```

- **Parameters.** Core allocates the Unix socket named by the service's `endpointParameter`
  (`socketPath`) in the private runtime directory of the Run; the process creates it there.
  The socket parameter remains `fixedOnly`. For the type servers and Tools, `runId` is the
  owning Run; it is part of the ROS node name (`xgc_scout_mini_${runId}`), so the processes
  of two Runs on one ROS master never share a node name, and every robot registered with the
  process must carry the same Run id. `rosMasterUri` is the Run's ROS master, `rosIp` the
  address of this host in the ROS network.
- **Services.** The type servers expose `xgc2.robot-adapter` with `grpc.v1`; Tools
  exposes `ros1.tools` with `http.v1`. Scene exposes `xgc2.scene.authoring` v1 on
  `socketPath` and `xgc2.sensor.lidar` v1 on `lidarSocketPath`, both with `http.v1`
  in every profile. Each service needs a unique, non-sensitive string endpoint
  parameter. Both Scene socket parameters are required and `fixedOnly`.
- **Readiness.** Kind `describe`: Core holds one outstanding Describe with `wait_ready_ms`
  (gRPC `Describe`, HTTP `GET /v1/describe?wait_ready_ms=N`). The service answers once its
  ROS node is registered with the master; Scene also validates its document and profile
  dependencies. Every declared service must answer Describe before the process
  is ready, including Scene's empty lidar service in `xsim` and `physical`.
  `startGraceMs` is the time the process may take
  before a failed connection counts; `timeoutMs` ends the wait.
- **Stop.** `rpc: Shutdown` asks Core to call the service's `Shutdown` first (drain, final zero
  commands, removal of the robots, exit); then SIGTERM, which does the same; SIGKILL after
  `graceMs`. The type servers need up to 23 s in the worst case (2 s commands in flight, 15 s
  robots, 0.5 s status subscribers, 5 s RPC drain), hence 25 s. The Tools process has no
  drain and 5 s. Scene stops through SIGTERM with a 5 s grace.
- **Run ownership.** A Run-owned process that ends fails its Run; there is no restart field.
  The exit status says why
  (type-servers.md): a changed or unreachable ROS master is 3 or 4.

The definition fields are `id`, `version`, `label`, `description`, `parameters`, `setupFiles`,
`command`, `services`, `readiness`, `stop` and `logs`. Readiness uses `kind: describe` with
`startGraceMs`/`timeoutMs`; stopping uses `graceMs` and, for type servers, `rpc: Shutdown`.
The schema id is unchanged.

## Differences between the definitions

The type server definitions differ in the provider, the executable and the node name prefix
(`xgc_px4_multirotor_`, `xgc_scout_mini_`, `xgc_mecanum_ugv_`). The Tools definition has no
`--provider` and no Shutdown RPC, takes `rosHostname` (mutually exclusive with `rosIp`) and
passes `--run-id` and `--node-name` the same way (`xgc_ros1_tools_${runId}`).

The Scene definition starts `xgc2_scene_adapter` directly with ROS remappings:
`__ns`, `__master` and `__ip` select the namespace and ROS environment, and `_scene_file`,
`_working_file`, `_generation_parameters_json`, `_save_directory`, `_target_id`,
`_xrpc_socket`, `_frozen`, `_profile` and `_simulation_service_ref_json` set the private
parameters. The lidar remappings are `_lidar_xrpc_socket`, `_lidar_enabled`
and `_lidar_defaults_json`, rendered from `lidarSocketPath`, `lidarEnabled`
and `lidarDefaultsJson`. The enabled/defaults values default to true and `{}`;
the socket has no default. `sessionJson` is a string defaulting to empty, named
by `command.stdinParameter`; `command.args` contains `--session-stdin`. It
carries the frozen Run session facts as one JSON document on stdin. Empty input
means no fleet; the executable reads only with the flag and validates a present
document before starting ROS or either socket. `sceneFile` is required;
the `gazebo` profile also requires a valid
`simulationServiceRefJson` at startup. The namespace defaults to `/xgc_scene` and the
node name is `xgc2_scene_adapter`. See [Scene adapter](scene-adapter.md) for the profiles,
topics, TF and CLI contract. Scene readiness holds the unbound
`GET /v1/describe?wait_ready_ms=N` on both sockets until ready or the wait ends,
for at most 30000 ms. With two services Scene uses SIGTERM and no `stop.rpc`.
Core computes no fleet manifest. The scene workflow binds `sessionJson` using
`{"runMode":"{{ $session.runMode }}","robots":{{ $session.robots }},"visualizationTopics":{{ $session.visualizationTopics }}}`.
These frozen values are copied unchanged. The adapter constructs the Gazebo
sampled fleet from the roster; xsim/physical retain the empty fleet. Legacy
node-level presets, topic patterns and file provisioning are not carried over.
The sensor topics are `<namespace>/simple_lidar/points` and, when requested,
`<namespace>/simple_lidar/beams`; the reference remains
`/xgc/scene/reference_cloud`.

## Checks

`test/test_process_definitions.py` checks every definition against the flags its executable
parses, the providers it serves and the executable path the package installs; the
installed-package gate (`.xgc2/scripts/check_installed_packages.sh`) checks the installed
files.
