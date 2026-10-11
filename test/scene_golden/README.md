# Scene golden corpus

This is the behavioral reference for the later C++ scene adapter replay harness.
The recorder calls the real `xgc2_scene_runtime` at scene commit **2940a99**.
It was recorded with **Python 3.8.10**, **PyYAML 5.3.1** (the libyaml loader and
dumper), and the read-only XRPC Python SDK **0.2.0**. Runtime implementation
source is neither copied nor modified. All scenarios execute in this process;
only the three committed generator fixture executables spawn child processes.
The runtime kills and waits for those generator process groups. The wire host
uses a private Unix socket, with explicitly closed clients, host, runtime and
joined workers. There is no ROS, Gazebo process, TCP connection or network call.

From this adapter worktree, set the two paths to the supplied read-only
checkouts (the scene root is the directory containing `src/` and `test/`):

```sh
SCENE_ROOT=/path/to/pinned/scene/runtime/scene_runtime
XRPC_PYTHON=/path/to/xrpc/python
PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$SCENE_ROOT/src:$XRPC_PYTHON" \
  python3.8 test/scene_golden/record.py --check \
  --scene-root "$SCENE_ROOT" --xrpc-python "$XRPC_PYTHON"
```

Omit `--check` to record the corpus and rebuild the coverage matrix. `--check`
records twice into independent temporary directories, compares both recordings
and the committed corpus byte for byte, and verifies this matrix against the
test inventory. A mismatch exits 1 and names each failing scenario. The corpus
is compact UTF-8 JSON, sorted by object key, with stable list order, finite JSON
numbers and one terminal newline. The recorder enforces a total below
3,000,000 bytes. It does not write bytecode into either source checkout.

## Replay format

Each `corpus/<category>/<scenario>.json` is independently replayable. Its
`scenario` is that category/name and `derived_from` names the Python tests as
`filename.py:test_function` (assembly instead names its two source functions).
`brief_boundaries`, when present, identifies explicitly requested boundary
inputs extending a test's intent. Requests, responses and file bytes are
captured from actual calls, rather than inferred expected values.

`setup.files` maps file paths to `{contents, mode}`. `contents` is exact UTF-8
text, including whitespace and final newlines; `mode` is a numeric POSIX mode
(for example 416 means octal 0640). A later observation with `contents: null`
means the file does not exist. `{symlink: target}` represents a link rather than
the target bytes. `setup.directories` lists directories that must exist before
replay. Parent directories of setup files must also be created. Root `$work`
always exists. File observations include all regular files and links under
that root and explicitly watched absent files; no observation implies a file
outside the granted tree should be created or changed.

`setup.executables`, when present, contains the entire generator fixture bytes
and executable modes. A replay can materialize them into its `$fixtures`
directory from this file alone. The committed executables are also provided
for direct inspection. They replace the tests' temporary scripts and inline
interpreter commands without changing their seeded output, counter, exit
status or output length. The generator path and counter path appear in the
recorded source YAML; their path-dependent SHA256 is bound as described below.

`setup.stores` maps a store label to its initial raw `document` and constructor
options: `source`, `source_digest`, `save_root`, `working_file`, `frozen`,
`apply_hook`, `motion_hook` and `clock`, when specified. Omitted options use
the runtime defaults. `clock: 100.0` denotes an injected fake monotonic clock;
all stores share it. The clock never advances except through an explicit
step. Hooks are enabled only when requested; otherwise apply and motion are
successful no-ops. `initial_response` is the exact initial envelope, and
`initial_effects` observes constructor hooks and files. Large store limit
fixtures omit the redundant initial envelope; their command response retains
it in full. Create stores in the recorded label order (`a`, `b`, or `main`).

`steps` is an ordered list. Each step has `operation`, exact `request`, and
`response`. For ordinary API calls the response is `{value: ...}`; an exception
is `{error: {type, message}}`, preserving its exact text. A store command's
unsuccessful result remains a value with `success: false`, not an exception.
An optional `store` selects the setup label. Each operation is defined here:

| Operation | Request and replay action | Response value |
| --- | --- | --- |
| `document.validate` | Raw document; call the canonical validator. | Accepted normalized document. |
| `document.geometry` | `{json}`; decode the raw JSON string (including nonfinite test literals) and validate geometry. | Normalized geometry. |
| `document.parse_json` | `{json}`; parse with duplicate field detection. | Parsed value. |
| `document.load` | `{path}`; load and validate disk YAML with duplicate detection. | `[normalized document, SHA256 of original file bytes]`. |
| `document.dump_yaml` | Normalized document; serialize with both the incremental writer and full dump. Reuse one incremental writer for this scenario. | `{incremental, full}` with exact YAML text. |
| `envelope.encode` | `{json}`; decode and run the envelope JSON encoder, reusing one encoder. | Exact JSON text (or nonfinite-number error). |
| `store.command` | Complete command object, including request ID and epoch/revision fences. | Complete command result/envelope. |
| `store.envelope` | `{}`; read the selected store. | Complete envelope. |
| `store.states` | `{}`; read state at the fake clock. | Ordered motion state list. |
| `clock.advance` | `{seconds}`; advance the injected monotonic clock. | New clock value. |
| `simulator.apply_outcome` | `{outcome: ok/rejected/unknown, message?}`; script all subsequent apply calls until changed. | `null`. |
| `file.mutate` | `{path, contents}` or `{path, symlink}`; mutate the file outside the store. | `null`. |
| `store.restart` | Exact constructor options including `initial`; construct or replace this label's store, reading `source` from disk first and honoring an existing working file. Other labels stay live. | New envelope with a new epoch. |
| `motion.state` | `{obstacle, elapsed, playing}`; pure motion evaluation of the normalized obstacle. | Exact pose and linear/angular velocity. |
| `generator.resolve` | `{source, working_file, overrides, max_bytes?}`; invoke source resolution. Omitted byte limit is the runtime's 8 MiB default. | `[[document, file digest], resolved path]`. |
| `gazebo.prepare` | Complete simulation prepare request; interpret setup files and hashes and materialize the world. | Exact receipt, parameters and hashes. |
| `assembly.cli` | `{stdin}`; provide those exact JSON bytes to the finite CLI. | `{exit_code, stdout, stderr}` with exact stream contents. |
| `http.call` | `{method, path, headers, body}`; send through the SDK to the real SceneService on the setup socket. | `{status, headers, body, body_text, client}`. |

`effects.files` observes resulting file contents and modes. `effects.apply`
and `effects.motion` are ordered native hook calls since the previous effect
capture. Apply requests contain the candidate document, epoch and reserved
revision; their responses contain the scripted outcome and optional message.
An apply observation also captures file bytes **at hook entry**, which proves
the first working copy exists before native application and contains the old
definition. Both rejected and unknown outcomes raise the specified SceneError
in the Python seam; unknown explicitly denotes the possibility that the
simulator changed before its reply was lost. The observed candidate remains
available to the replay harness. Motion observations contain operation,
epoch and revision with an `ok` outcome.

A command step's optional `fault: {operation: file.replace, message}` makes
every atomic rename in that command fail with `OSError(message)`, then restores
normal IO. This reproduces the tests' disk-full injection. It is not a document
or transport option. No other fault injection is used.

Assembly `stdout` is the exact CLI JSON, without an added newline. Failures
exit 2 and write one stderr line. A replay should also compare the YAML scene
embedded in stdout. Twelve assembly scenarios exercise both fidelities,
nonzero x/y/yaw, circle phase, rotated motion points and velocities, and local
validation and parsing errors. The two explicit byte-size guards (input over
32 MiB; assembled output over 8 MiB) cannot be represented as literal requests
inside a corpus below 3 MB. They remain later C++ boundary tests; no repetition,
compression or input-size override language is introduced to hide this conflict.
Canonical validator rejection texts are shared with the document category;
assembly is not an exhaustive duplicate of that validator.

Wire setup supplies the service target, socket, online state and observed
service reference. Request headers include `X-Request-ID`, `Content-Type` for
JSON posts, and the instance header for bound calls. The SDK supplies its
bounded timeout header; its remaining millisecond budget is intentionally not
part of the golden bytes. The response retains `Content-Type`,
`X-Xrpc-Instance-ID` when present, and `X-Request-ID`, plus the exact body text
and parsed body. Other HTTP library headers (Date, Server, framing) are omitted.
`client` preserves disposition and any SDK Fault/TransportError text, code and
status. Capture uses the SDK's native HTTPX transport seam and real Unix IO,
including the 409 reply rejected by the stale-instance client. A conflicting
command identity is a 200 store rejection; a payload `requestId` is a 400
XRPC fault; an unknown receipt is 404. Unbound describe, including
`?wait_ready_ms=10`, is discovery by path and succeeds without an instance.
`cleanup` observes socket removal and runtime closure after the scenario.

## Token bindings

Substitute only these literal tokens, including inside strings and object
keys, matching the longest token first (so `$epoch1` precedes `$epoch`).
There are no expressions, macros, repetition directives or references
to other steps. Bind tokens before calls; normalize observed responses using
the reverse bindings before comparison. A token may appear inside YAML, XML,
CLI JSON text, wire body text, a message or a filesystem path.

| Token | Binding and all occurrence locations |
| --- | --- |
| `$work` | Scenario's fresh absolute temporary root. Setup paths, constructor options, restart requests, file mutation/load/resolve/prepare requests, disk observations, generator argv/counter, service socket/reference and prepared plugin XML. |
| `$fixtures` | Absolute directory for materialized fixture executables. Setup executable keys, generator command argv and source YAML file contents. |
| `$epoch` | First store's generated epoch. All envelopes, command fences, status configuration, native hook requests, wire request/response bodies and their encoded JSON. |
| `$epoch1`, `$epoch2`, `$epoch3` | Subsequent store/restart epochs in creation order within this scenario; same locations as `$epoch`. Other simultaneously live stores retain their previous binding. |
| `$instance` | SceneService's random instance ID. Service reference, bound request header, response instance header, describe body and its encoded JSON. The deliberate stale value is the literal `stale`. |
| `$sourceSha256` | SHA256 of the random source YAML's exact bytes **after** path substitution. Generation provenance YAML, the random prepare manifest's sceneDocument hash and its observed file contents. The original source in the random prepare scenario is replaced by a file mutation first. |
| `$preparedSha256` | SHA256 of the prepared-world file's exact bytes after `$work` substitution. Prepare receipt's `preparedWorldSha256`. Its binding is computed from the observed/expected world bytes; compare those bytes as well. |

All other digests are literal SHA256 values. Monotonic time is fixed or
explicitly advanced, so it needs no token. Request IDs and run/target identities
are scripted literals. No random identity is replaced with a fixed one inside
the runtime: normalization happens after observation.

## Coverage

The table inventory comes from `grep -n "def test_" "$SCENE_ROOT"/test/*.py`.
Every test has a row; partial coverage states the excluded intent. The consumer
registry and topic acknowledgement heartbeats are dropped by the binding
decision. Native applied/known/epoch/revision status is retained in store
scenarios. ROS node messages, markers, TF and launch forwarding are excluded.
Display names from test fixtures are English here. The non-ASCII roster
rejection uses a Latin character instead of the test's original text. The
explicit document ID and 129-part boundaries are required by this package's
brief and marked as boundaries beyond the related test's original inputs.

<!-- coverage:start -->
<!-- coverage:end -->
