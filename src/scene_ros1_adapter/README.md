# xgc_scene_ros1_adapter

The Run-owned scene authoring adapter exposes `xgc2.scene.authoring` v1 over
XRPC `http.v1` on a Unix socket. `xgc2_scene_adapter` publishes scene snapshots,
state, envelope JSON, markers and TF. Profiles are `gazebo`, `xsim` and `physical`.
The finite tools are `xgc2-scene-assemble` and `xgc2-scene-prepare`.

[Scene adapter](../../docs/scene-adapter.md) documents the profiles, private ROS
parameters, topics, TF and CLI usage. The Debian package is
`ros-noetic-xgc2-scene-adapter`; its process definition is
`process/xgc2-ros1-scene-adapter.json`.

`xgc_scene_core` (`src/core`, `include/xgc_scene`) is the ROS-free static authoring
library: document validation, YAML/JSON, motion, assembly, generation, preparation
and persistence. It uses nlohmann-json, yaml-cpp, OpenSSL and `xgc2_math::scene`
(Math 0.6.0 or newer). Simulators own their sensor implementations.

Core sources (`src/core/*.cpp`) and tests (`test/core/*_test.cpp`) are globbed.
The golden corpus (`test/scene_golden` at the repository root) is the reference.

```sh
cmake -S src/scene_ros1_adapter -B build -DCMAKE_PREFIX_PATH=/path/to/math-prefix
cmake --build build -j2
(cd build && ctest --output-on-failure -j2)
```
