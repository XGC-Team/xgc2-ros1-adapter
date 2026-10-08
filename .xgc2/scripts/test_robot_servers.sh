#!/usr/bin/env bash
set -euo pipefail

# Invoke only in the caller's disposable ROS container, after installing the
# three real Debian packages. The fixture owns its master and child processes.
if [[ $# != 2 ]]; then
  echo "usage: $0 <catkin-build-directory> <empty-private-results-directory>" >&2
  exit 2
fi
if [[ "${ROBOT_SERVER_PRIVATE_TEST:-}" != 1 ]]; then
  echo "requires an isolated ROS test environment with ROBOT_SERVER_PRIVATE_TEST=1; the fixture creates and verifies its own random-port master" >&2
  exit 2
fi
set +u
source /opt/ros/noetic/setup.bash
set -u
python3 -c 'import grpc, rospy, google.protobuf'
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
build_root="$(realpath "$1")"
results="$(realpath -m "$2")"
mkdir "${results}"
parallel_jobs="${BUILD_JOBS:-2}"
cmake --build "${build_root}" --target robot_server_test_tools -- -j"${parallel_jobs}"
devel_root="$(sed -n 's/^CATKIN_DEVEL_PREFIX:PATH=//p' "${build_root}/CMakeCache.txt")"
if [[ -z "${devel_root}" ]]; then
  devel_root="$(sed -n 's/^CATKIN_DEVEL_PREFIX:UNINITIALIZED=//p' "${build_root}/CMakeCache.txt")"
fi
for tool in async_ros_services_driver ros_forwarding_source ros_forwarding_observer; do
  ln -s "${devel_root}/lib/robot_server_tests/${tool}" "${results}/${tool}"
done
proto_root="${XGC2_PROTOBUF_PROTO_ROOT:-/usr/share/xgc2-protobuf/proto}"
mkdir "${results}/python"
mapfile -t proto_files < <(find "${proto_root}/xgc" -name '*.proto' -type f | sort)
protoc --proto_path="${proto_root}" --python_out="${results}/python" "${proto_files[@]}"
export PYTHONPATH="${results}/python${PYTHONPATH:+:${PYTHONPATH}}"
export ASYNC_ROS_SERVICES_DRIVER="${results}/async_ros_services_driver"
export ROBOT_SERVER_BUILD_ROOT="${results}"
export ROBOT_SERVER_INSTALL_PREFIX="${ROBOT_SERVER_INSTALL_PREFIX:-/opt/ros/noetic}"
# The all-robot ROS fixtures own hundreds of actual TCP connections.
ulimit -n 4096
python3 "${repo_root}/common/test/test_async_ros_services.py" 2>&1 | tee "${results}/transport.log"
python3 "${repo_root}/common/test/test_robot_server.py" 2>&1 | tee "${results}/servers.log"
if [[ -n "${ROBOT_SERVER_CORE_TEST_BINARY:-}" ]]; then
  XGC_ROBOT_SERVER_NATIVE_TEST=1 "${ROBOT_SERVER_CORE_TEST_BINARY}" \
    -test.run '^TestManagerNativeStateScale$' -test.v -test.timeout 240s \
    2>&1 | tee "${results}/core-client.log"
fi
