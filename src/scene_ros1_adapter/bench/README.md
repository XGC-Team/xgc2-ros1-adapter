# Lidar fleet evidence

Build this optional executable from the scene adapter's plain CMake project:

```sh
cmake -S src/scene_ros1_adapter -B /home/node/workspace/build/lf-engine/release \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH=/home/node/workspace/build/prefix-math-scene \
  -DCMAKE_DISABLE_FIND_PACKAGE_catkin=TRUE \
  -DXGC_SCENE_BUILD_BENCH=ON \
  -DXGC_SCENE_OLD_FLEET_SOURCE=/home/node/workspace/xgc2-harness/devops/products/ros1/common/scene/sensors/world_lidar
cmake --build /home/node/workspace/build/lf-engine/release -j1
nice -n 19 /home/node/workspace/build/lf-engine/release/bench_lidar_fleet 1 1
```

The source directory must be the read-only scene checkout at `2940a99`. CMake
compiles its original `src/scan_pool.cpp` directly, without copying or changing
it. Both paths use the installed Math `WorldLidar`; this measures the fleet
implementation independently of the earlier geometry migration.

The baseline in `fleet.cpp` follows `world_lidar_node.cpp`'s fresh/active vectors,
per-sensor objects, simulation schedules, scene-policy cache, `scanInto` points
and allocating beam publication path. ROS serialization and transport are
outside this comparison. Both implementations deliver to the same nonallocating
sink, which hashes every byte during delivery. Checksums and effective point,
beam and scan counts must match, or the executable fails.

The fixed seed constructs 600 obstacles through Math's cluttered-environment
descriptions: 120 each of spheres, cylinders, capsules, boxes and convex
polytopes. Sensors move across a grid by 0.075 m per tick. Each fleet has 20 or
100 sensors, range 8 m, 10 Hz, and runs 100 simulation ticks after one warm scan.
Penetrating scans use spacing 0.1 m and keep buried samples. Raycast scans use
360 by 32 beams over a full sphere and publish both points and beams. All
sensors have demand. Noise and body sensing are off in the benchmark; the
allocation and equivalence tests cover both features separately.

The separate `body_candidate_growth` probe moves one roster body from outside
range to inside range after the first scan. In the required Math prefix this
first arrival allocates in `WorldLidar`'s private `near_` vector; the following
tick allocates nothing. Math exposes no way to reserve that scratch from the
engine. This is an explicit exception to an absolute first-scan allocation
guarantee for arbitrary body motion, and is recorded in the package report.

The arguments are repetitions and scan threads; both default to one. A scan
thread count of zero selects the original hardware-based default for both
paths. The single-thread command above follows the foreman's emergency load
limits. Further parallel evidence collection is deferred to CI.
The authorized revision comparison uses `bench_lidar_fleet 5 1`: five
repetitions in one single-thread process, with old/engine order alternating.
Each repetition alternates the order of the old and engine paths. Output is
CSV with column descriptions. Wall mean is measured around `tick`; process CPU
time includes all workers. Last-delivery p50/p99/max measure the interval from
tick entry through the last sink callback, including byte hashing. Allocation
counts include every worker and all global C++ allocation forms. An engine
allocation after warm-up fails the executable. Default parallelism includes the
calling thread; it starts one fewer background workers. Every pool is destroyed
and joined before the next path is constructed.

After each fleet run, 40 revisions move five obstacles by 0.05 m each. The old
policy-map installation and the engine's flat policy cache both use Math's
incremental scene constructor and install into the same number of sensors.
Revision timings include installation; engine sensor preparation happens
lazily on its next demanded scan. Individual `revision_sample` rows allow
pooling all 200 samples per path/mode/fleet size to report median and p99. The
acceptance is engine median wall time less than or equal to the old median in
each of the four cases, without a tolerance or discarded samples. Host load
averages are printed before and after each repetition. On a shared host, retain
all repetitions and report medians and their load context.
