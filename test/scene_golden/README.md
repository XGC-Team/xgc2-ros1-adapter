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

## Intentional preparation divergence

The coordinator decision of 2026-10-11 intentionally changes preparation from
the pinned Python behavior. Files under `corpus/`, including their inputs,
outputs and coverage matrix, remain the actual Python recordings. `record.py`
records and compares only `corpus/`; `--check` still verifies those recordings
without applying the divergence or claiming that Python implements it.

For recorded preparation scenarios, the C++ replay harness applies one explicit
expectation rule, leaving requests unchanged:

- A request containing `chassis_robot_ids` expects `SceneError` with the message
  `scene parameter chassis_robot_ids is no longer accepted; the Gazebo chassis HOLD roster is dynamic`.
  It produces no prepared world. Later failed requests retain the last successfully
  prepared world, or its absence. In `explicit-rosters`, the first request now fails,
  so its recorded prepared world is absent throughout the scenario.
- A successful request omitting `required_components` expects `["vrpn"]` in the
  receipt and one `<required_component>vrpn</required_component>` immediately before
  `<clock_rate_hz>` in the native plugin. Explicit lists remain unchanged. Successful
  receipts omit `parameters.chassis_robot_ids`. The existing `$preparedSha256`
  binding hashes the adjusted expected bytes after path substitution.

`authored/prepare/` contains four hand-written scenarios in the same JSON format:
`default-vrpn`, `empty-components`, `other-component` and `legacy-chassis-parameter`.
Their provenance explicitly identifies the coordinator decision, not a Python
test recording. They reuse the native-geometry asset fixture and replay literal
expectations without adjustment through both the library and the real CLI.
The legacy scenario also covers empty/null values and precedence over an unknown
key. Full receipts, file bytes, file modes and exact CLI error lines are checked.

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
| Python test case | Source line | Coverage or explicit exclusion |
| --- | ---: | --- |
| `test_prepare.py:test_native_world_keeps_exact_geometry_and_owns_only_its_granted_output` | 36 | [prepare/native-world.json](corpus/prepare/native-world.json) |
| `test_prepare.py:test_document_editability_and_media_attachments_are_provider_owned` | 54 | [prepare/document-and-media.json](corpus/prepare/document-and-media.json) |
| `test_prepare.py:test_wrong_revision_bad_hash_unknown_physics_and_symlink_escape_are_rejected` | 72 | [prepare/revision-hash-parameters-grant.json](corpus/prepare/revision-hash-parameters-grant.json) |
| `test_prepare.py:test_random_generator_output_is_bounded_before_world_commit` | 85 | [generation/output-limit.json](corpus/generation/output-limit.json), [prepare/random-output-limit.json](corpus/prepare/random-output-limit.json) |
| `test_prepare.py:test_explicit_chassis_and_component_rosters_are_not_inferred` | 100 | [prepare/explicit-rosters.json](corpus/prepare/explicit-rosters.json) |
| `test_prepare.py:test_asset_attachment_symlink_cannot_escape_its_asset_directory` | 113 | [prepare/attachment-symlink.json](corpus/prepare/attachment-symlink.json) |
| `test_scene_consumers.py:test_success_is_not_an_applied_authority` | 14 | Dropped by decision: consumer registry and ROS-topic acknowledgement heartbeats. |
| `test_scene_consumers.py:test_capability_gap_does_not_count_as_retryable_unsync` | 26 | Dropped by decision: consumer registry and ROS-topic acknowledgement heartbeats. |
| `test_scene_consumers.py:test_version_lag_and_apply_failure_remain_retryable` | 36 | Dropped by decision: consumer registry and ROS-topic acknowledgement heartbeats. |
| `test_scene_consumers.py:test_applied_capability_limit_still_counts_as_document_sync` | 45 | Dropped by decision: consumer registry and ROS-topic acknowledgement heartbeats. |
| `test_scene_consumers.py:test_exited_consumers_expire_and_reentry_replaces_them` | 55 | Dropped by decision: consumer registry and ROS-topic acknowledgement heartbeats. |
| `test_scene_consumers.py:test_old_epoch_and_out_of_order_stamps_do_not_overwrite` | 71 | Dropped by decision: consumer registry and ROS-topic acknowledgement heartbeats. |
| `test_scene_consumers.py:test_public_envelope_splits_sync_from_capability_and_derives_success` | 87 | Dropped by decision: consumer registry and ROS-topic acknowledgement heartbeats. |
| `test_scene_frozen.py:test_every_edit_save_and_reload_channel_is_rejected_with_an_explicit_error` | 29 | [store/frozen-rejections.json](corpus/store/frozen-rejections.json) |
| `test_scene_frozen.py:test_rejection_is_a_result_not_a_silent_drop_or_exception` | 49 | [store/frozen-rejections.json](corpus/store/frozen-rejections.json) |
| `test_scene_frozen.py:test_frozen_scene_still_serves_reads_motion_clock_and_resync` | 59 | [store/frozen-read-motion-resync.json](corpus/store/frozen-read-motion-resync.json) |
| `test_scene_frozen.py:test_frozen_source_yaml_is_never_reloaded_or_written` | 72 | [store/frozen-source.json](corpus/store/frozen-source.json) |
| `test_scene_frozen.py:test_unfrozen_default_keeps_every_existing_channel_open` | 85 | [store/external-edit-reload.json](corpus/store/external-edit-reload.json) |
| `test_scene_frozen.py:test_scene_launch_forwards_the_frozen_argument_as_a_param` | 100 | ROS-bound launch forwarding; to be written as a C++ launch contract test from the Python intent. |
| `test_scene_generation.py:test_external_generator_runs_once_and_saves_exact_parameters` | 11 | [generation/external-once.json](corpus/generation/external-once.json) |
| `test_scene_generation.py:test_generator_failure_does_not_publish_a_scene` | 33 | [generation/failure-no-publish.json](corpus/generation/failure-no-publish.json) |
| `test_scene_projection.py:test_equals_json_dumps` | 153 | [document/envelope-json.json](corpus/document/envelope-json.json); Plain encoder cases only; consumer fields are dropped. JSON converts integer object keys to strings; cache branch identity is a later C++ test concern. |
| `test_scene_projection.py:test_snapshots_markers_and_states_equal_the_uncached_projections` | 187 | [motion/rotating-twist.json](corpus/motion/rotating-twist.json); ROS-bound snapshots/markers/state projections are excluded; motion/rotating-twist records only the plain motion sample. To be written as C++ ROS output tests. |
| `test_scene_scaling.py:test_gazebo_receives_one_batched_apply_per_revision` | 175 | ROS-bound node/application batching; to be written as a C++ test from the Python intent. |
| `test_scene_scaling.py:test_unchanged_consumer_heartbeats_do_not_republish_the_document` | 181 | Dropped by decision: consumer registry and ROS-topic acknowledgement heartbeats. |
| `test_scene_scaling.py:test_native_failure_and_recovery_change_the_published_view` | 187 | ROS-bound published synchronization view; store/native-failure and store/lost-reply-resync retain native applied status. To be written as a C++ output test. |
| `test_scene_scaling.py:test_tick_and_definition_scan_the_obstacle_list_a_constant_number_of_times` | 202 | Performance-only obstacle scan counts (also ROS-bound). |
| `test_scene_store.py:test_obstacle_visual_color_is_product_amber` | 30 | [document/visual-color.json](corpus/document/visual-color.json) |
| `test_scene_store.py:test_compounds_preserve_parts_and_geometry_has_explicit_sizes` | 39 | [document/compound-sizes.json](corpus/document/compound-sizes.json) |
| `test_scene_store.py:test_invalid_geometry_does_not_become_another_shape` | 49 | [document/invalid-geometry.json](corpus/document/invalid-geometry.json) |
| `test_scene_store.py:test_duplicate_fields_and_huge_numbers_are_rejected` | 57 | [document/duplicate-and-huge.json](corpus/document/duplicate-and-huge.json) |
| `test_scene_store.py:test_convex_manifold_required_without_filling_compound_holes` | 68 | [document/convex-manifold.json](corpus/document/convex-manifold.json) |
| `test_scene_store.py:test_ids_are_stable_through_delete_copy_undo_and_reload` | 77 | [document/identifiers.json](corpus/document/identifiers.json), [motion/hold.json](corpus/motion/hold.json), [store/stable-ids-undo-redo.json](corpus/store/stable-ids-undo-redo.json) |
| `test_scene_store.py:test_stale_epoch_revision_and_request_reuse_cannot_overwrite` | 88 | [store/identity-and-stale.json](corpus/store/identity-and-stale.json) |
| `test_scene_store.py:test_simulator_rejection_does_not_commit_or_create_undo_entry` | 99 | [store/simulator-rejected.json](corpus/store/simulator-rejected.json) |
| `test_scene_store.py:test_edits_undo_and_redo_automatically_survive_restart` | 112 | [store/autosave-restart.json](corpus/store/autosave-restart.json) |
| `test_scene_store.py:test_agent_file_edits_require_reload_and_are_never_overwritten` | 126 | [store/external-edit-reload.json](corpus/store/external-edit-reload.json) |
| `test_scene_store.py:test_save_failure_keeps_live_edit_and_retry_does_not_apply_it_twice` | 145 | [store/save-failure-retry.json](corpus/store/save-failure-retry.json) |
| `test_scene_store.py:test_lost_reply_and_partial_apply_recover_without_reusing_revision` | 164 | [store/lost-reply-resync.json](corpus/store/lost-reply-resync.json) |
| `test_scene_store.py:test_source_changed_during_startup_is_not_overwritten` | 184 | [store/startup-digest.json](corpus/store/startup-digest.json) |
| `test_scene_store.py:test_motion_clock_and_saved_initial_definition_remain_separate` | 193 | [motion/constant-twist.json](corpus/motion/constant-twist.json), [store/motion-clock.json](corpus/store/motion-clock.json) |
| `test_scene_store.py:test_motion_path_and_initial_pose_cannot_disagree` | 210 | [document/motion-start.json](corpus/document/motion-start.json), [motion/circle.json](corpus/motion/circle.json), [motion/ping-pong.json](corpus/motion/ping-pong.json) |
| `test_scene_store.py:test_full_snapshot_can_be_empty_but_bad_document_cannot_clear_it` | 223 | [document/empty-snapshot.json](corpus/document/empty-snapshot.json), [store/empty-snapshot.json](corpus/store/empty-snapshot.json) |
| `test_scene_store.py:test_experiment_working_files_are_isolated_and_persist_edits` | 230 | [store/working-isolation.json](corpus/store/working-isolation.json) |
| `test_scene_store.py:test_working_file_is_created_on_first_edit_before_apply` | 269 | [store/working-before-apply.json](corpus/store/working-before-apply.json) |
| `test_scene_store.py:test_first_working_copy_write_failure_does_not_apply_or_change_source` | 297 | [store/working-copy-failure.json](corpus/store/working-copy-failure.json) |
| `test_scene_store.py:test_existing_working_file_wins_over_changed_source_on_reopen` | 323 | [store/working-reopen.json](corpus/store/working-reopen.json) |
| `test_scene_store.py:test_incremental_yaml_equals_a_full_dump` | 366 | [motion/circle-phase.json](corpus/motion/circle-phase.json), [store/incremental-yaml.json](corpus/store/incremental-yaml.json) |
| `test_scene_store.py:test_one_edit_validates_and_emits_only_the_edited_obstacle` | 374 | Performance-only validation/emission counts and Python object identity; to be written as a C++ immutability/cost test from the intent. Store undo and YAML behavior are recorded separately. |
| `test_scene_store.py:test_scene_limits_still_apply_to_incremental_edits` | 401 | [document/limits.json](corpus/document/limits.json), [store/duplicate-obstacle.json](corpus/store/duplicate-obstacle.json), [store/obstacle-limit.json](corpus/store/obstacle-limit.json), [store/part-limit.json](corpus/store/part-limit.json) |
| `test_xrpc_control.py:test_real_authoring_host_and_retained_receipts` | 28 | [wire/authoring-http.json](corpus/wire/authoring-http.json) |
| `test_xrpc_control.py:test_simulation_waits_once_and_never_probes_entities` | 56 | Simulation-client transport sequencing; to be written as a C++ client integration test from the Python intent. |
| `test_xrpc_control.py:test_failed_native_effect_does_not_promote_desired_or_persisted` | 83 | [store/native-failure.json](corpus/store/native-failure.json) |
<!-- coverage:end -->
