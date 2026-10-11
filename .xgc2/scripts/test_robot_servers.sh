#!/usr/bin/env bash
set -euo pipefail

# Runs the integration tests against the installed packages: the installed type
# servers and Tools process, started the way a Run starts them, against a ROS
# master of their own (random port, private runtime and ROS home directories).
# The test client is built in the catkin workspace of this checkout.
if [[ $# != 2 ]]; then
  echo "usage: $0 <catkin-build-directory> <empty-results-directory>" >&2
  exit 2
fi
set +u
source /opt/ros/noetic/setup.bash
set -u
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
build_root="$(realpath "$1")"
results="$(realpath -m "$2")"
mkdir "${results}"
parallel_jobs="${BUILD_JOBS:-2}"
cmake --build "${build_root}" --target robot_server_client async_ros_services_driver \
  -- -j"${parallel_jobs}"
devel_root="$(sed -n 's/^CATKIN_DEVEL_PREFIX:PATH=//p' "${build_root}/CMakeCache.txt")"
if [[ -z "${devel_root}" ]]; then
  devel_root="$(sed -n 's/^CATKIN_DEVEL_PREFIX:UNINITIALIZED=//p' "${build_root}/CMakeCache.txt")"
fi
test -x "${devel_root}/lib/xgc_ros1_adapter_common/robot_server_client"
test -x "${devel_root}/lib/xgc_ros1_adapter_common/async_ros_services_driver"

export XGC_TEST_DEVEL="${devel_root}"
export XGC_TEST_INSTALL_ROOT="${ROBOT_SERVER_INSTALL_ROOT:-/}"
export XGC_TEST_REGISTRY="${XGC2_PROTOBUF_REGISTRY_JSON:-/usr/share/xgc2-protobuf/registry/registry.json}"
# The installed-package release cell owns the tagged 60-second outage check.
export XGC_TEST_LONG=1
# Hundreds of TCPROS connections with 100 robots.
ulimit -n 4096
# The TCPROS service client against a controlled transport fixture.
ASYNC_ROS_SERVICES_DRIVER="${devel_root}/lib/xgc_ros1_adapter_common/async_ros_services_driver" \
  python3 "${repo_root}/src/common/test/test_async_ros_services.py" 2>&1 \
  | tee "${results}/transport.log"
python3 -m unittest discover -v -s "${repo_root}/test/integration" -p 'test_*.py' 2>&1 \
  | tee "${results}/integration.log"
