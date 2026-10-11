# xgc_scene_ros1_adapter

The scene adapter of the ROS1 services: the scene authoring service (`xgc2.scene.authoring`, http.v1), its Gazebo application and the
Gazebo lidar fleet (`xgc2.sensor.lidar`), started and ended by the Run. Design record: specs/scene-interfaces.md section 1.

| Library | Content | Needs |
| --- | --- | --- |
| `xgc_scene_core` (`src/core`, `include/xgc_scene`) | document model and validation, YAML/JSON primitives, motion, assembly, generation, Gazebo preparation, the store | nlohmann-json, yaml-cpp, OpenSSL, `xgc2_math::scene` |
| `xgc_scene_lidar` (`src/lidar`, `include/xgc_scene/lidar`) | the Gazebo lidar fleet engine: manifest, scene cache, scheduler, bounded pool, reused buffers | nlohmann-json, `xgc2_math::scene` |

Both libraries are ROS-free. ROS-bound code (service shell, publishers, subscriptions) is added next to them. Sources (`src/core/*.cpp`,
`src/lidar/*.cpp`) and tests (`test/core/*_test.cpp`, `test/lidar/*_test.cpp`) are globbed. The golden corpus of the Python scene service
(`test/scene_golden` at the repository root) is the reference of the core tests.

```sh
cmake -S src/scene_ros1_adapter -B build -DCMAKE_PREFIX_PATH=<prefix with xgc2_math 0.6.0> && cmake --build build -j2 && ctest --test-dir build
```
