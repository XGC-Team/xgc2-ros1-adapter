#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-noetic}"
PREFIX="/opt/ros/${ROS_DISTRO}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
COMMON_PACKAGE="ros-${ROS_DISTRO}-xgc2-ros1-adapter-common"
COMMON_ROS_PACKAGE="xgc_ros1_adapter_common"
PX4_PACKAGE="ros-${ROS_DISTRO}-xgc2-px4-multirotor-adapter"
PX4_ROS_PACKAGE="xgc_px4_multirotor_ros1_adapter"
UGV_PACKAGE="ros-${ROS_DISTRO}-xgc2-ugv-adapter"
UGV_ROS_PACKAGE="xgc_ugv_ros1_adapter"
TOOLS_PACKAGE="ros-${ROS_DISTRO}-xgc2-ros1-tools-adapter"
TOOLS_ROS_PACKAGE="xgc_ros1_tools_adapter"
SCENE_PACKAGE="ros-${ROS_DISTRO}-xgc2-scene-adapter"
SCENE_ROS_PACKAGE="xgc_scene_ros1_adapter"
EXPECTED_PRODUCT_VERSION="${EXPECTED_PRODUCT_VERSION:-$(
  awk -F': *' '/^version:[[:space:]]*/ {print $2; exit}' \
    "${REPO_ROOT}/.xgc2/product.yml"
)}"
PROTOBUF_REGISTRY="/usr/share/xgc2-protobuf/registry/registry.json"

for package in "${COMMON_PACKAGE}" "${PX4_PACKAGE}" "${UGV_PACKAGE}" "${TOOLS_PACKAGE}" "${SCENE_PACKAGE}"; do
  dpkg -s "${package}" >/dev/null
  test "$(dpkg-query -W -f='${Version}' "${package}")" = "${EXPECTED_PRODUCT_VERSION}"
done
test -f "${PROTOBUF_REGISTRY}"
test -x "${PREFIX}/lib/${COMMON_ROS_PACKAGE}/xgc2-project-robot-assets"
test "$(printf '%s' '{"context":{},"robots":[]}' | "${PREFIX}/lib/${COMMON_ROS_PACKAGE}/xgc2-project-robot-assets")" = '[]'
# Scout Mini and Mecanum share one executable and one package now; the
# earlier per-type packages and the generic adapter must be gone.
for removed in \
    "ros-${ROS_DISTRO}-xgc2-ros1-adapter" \
    "ros-${ROS_DISTRO}-xgc2-scout-mini-adapter" \
    "ros-${ROS_DISTRO}-xgc2-mecanum-ugv-adapter"; do
  if dpkg -s "${removed}" >/dev/null 2>&1; then
    echo "removed package ${removed} is still installed" >&2
    exit 1
  fi
done

common_depends="$(dpkg-query -W -f='${Depends}' "${COMMON_PACKAGE}")"
px4_depends="$(dpkg-query -W -f='${Depends}' "${PX4_PACKAGE}")"
ugv_depends="$(dpkg-query -W -f='${Depends}' "${UGV_PACKAGE}")"
grep -Eq '(^|, )libgrpc\+\+1( |[(,]|$)' <<<"${common_depends}"
grep -Eq '(^|, )libc-ares2( |[(,]|$)' <<<"${common_depends}"
grep -Eq '(^|, )libxgc2-xrpc-grpc2( |[(,]|$)' <<<"${common_depends}"
for depends in "${common_depends}" "${px4_depends}" "${ugv_depends}"; do
  if grep -Eq 'libxgc2-adapter-runtime|xgc2-protobuf-dev|libxgc2-xrpc-(grpc-)?dev' <<<"${depends}"; then
    echo "robot type server package depends on a build-only or removed package" >&2
    exit 1
  fi
done
for depends in "${px4_depends}" "${ugv_depends}"; do
  grep -Fq "${COMMON_PACKAGE} (= ${EXPECTED_PRODUCT_VERSION})" <<<"${depends}"
done
grep -q "ros-${ROS_DISTRO}-mavros-msgs" <<<"${px4_depends}"
if grep -Eq 'mavros-msgs|scout-msgs' <<<"${ugv_depends}"; then
  echo "UGV adapter package must not depend on MAVROS or Scout messages" >&2
  exit 1
fi
if grep -q 'scout-msgs' <<<"${px4_depends}"; then
  echo "PX4 adapter package must not depend on Scout messages" >&2
  exit 1
fi

set +u
# shellcheck disable=SC1090
source "${PREFIX}/setup.bash"
set -u
dpkg -s "ros-${ROS_DISTRO}-roscpp" >/dev/null

# The shared libraries belong to the common package.
for library in libxgc2_robot_server.so.0 libxgc2_robot_server_ros1.so.0; do
  test -e "${PREFIX}/lib/${library}"
  dpkg-query -S "${PREFIX}/lib/${library}" | grep -Fq "${COMMON_PACKAGE}:"
done
rospack find "${COMMON_ROS_PACKAGE}" >/dev/null

# check_type_server <ros package> <package> <provider>...
check_type_server() {
  local ros_package="$1"
  local package="$2"
  shift 2
  local executable="${PREFIX}/lib/${ros_package}/${ros_package}_node"
  local share="${PREFIX}/share/${ros_package}"
  local provider
  local definition
  local libraries
  local needed
  local profile_name

  rospack find "${ros_package}" >/dev/null
  test -f "${share}/package.xml"
  test -f "${share}/profiles/schema/robot-adapter-profile-v4.schema.json"
  test ! -e "${share}/launch"
  test ! -e "${PREFIX}/include/${ros_package}"
  test -x "${executable}"
  dpkg-query -S "${executable}" | grep -Fq "${package}:"

  libraries="$(ldd "${executable}")"
  if grep -q 'not found' <<<"${libraries}"; then
    echo "${executable} has unresolved libraries" >&2
    echo "${libraries}" >&2
    exit 1
  fi
  grep -q 'libxgc2_robot_server_ros1' <<<"${libraries}"
  grep -q 'libgrpc++' <<<"${libraries}"
  grep -q 'libcares' <<<"${libraries}"
  # The bounded ros::master::execute lives in libxgc2_robot_server_ros1 and
  # must come ahead of roscpp in the executable's own dependency list.
  needed="$(readelf -d "${executable}" | sed -n 's/.*Shared library: \[\(.*\)\]/\1/p')"
  test "$(grep -n '^libxgc2_robot_server_ros1' <<<"${needed}" | head -1 | cut -d: -f1)" -lt \
       "$(grep -n '^libroscpp' <<<"${needed}" | head -1 | cut -d: -f1)"

  for provider in "$@"; do
    definition="/usr/share/xgc2/process-definitions/${provider}.json"
    test -f "${definition}"
    dpkg-query -S "${definition}" | grep -Fq "${package}:"
    test -f "/usr/share/xgc2/robot-adapter-profiles/${provider}.json"
    cmp "${definition}" "${REPO_ROOT}/src/${ros_package#xgc_}/process/${provider}.json"
    python3 - "${definition}" "${executable}" "${provider}" <<'PYTHON'
import json
import os
import sys

path, executable, provider = sys.argv[1:]
document = json.load(open(path))
assert document["apiVersion"] == "xgc.execution.process/v1", document["apiVersion"]
(definition,) = document["definitions"]
assert set(definition) <= {"id", "version", "label", "description", "parameters", "setupFiles", "command", "services", "readiness", "stop", "logs"}
assert definition["id"] == provider
assert definition["command"]["executable"] == executable
assert os.access(executable, os.X_OK)
assert definition["services"][0]["service"] == "xgc.robot.v1.RobotAdapterServerService"
assert definition["services"][0]["profile"] == "grpc.v1"
assert set(definition["services"][0]) == {"service", "api_version", "profile", "endpointParameter"}
assert set(definition["parameters"]) <= {"properties", "required", "groups"}
for parameter in definition["parameters"]["properties"].values():
    assert set(parameter) <= {"type", "default", "enum", "minimum", "maximum", "sensitive", "output", "fixedOnly"}
endpoint = definition["services"][0]["endpointParameter"]
assert definition["parameters"]["properties"][endpoint] == {"type": "string", "fixedOnly": True}
assert definition["readiness"]["kind"] == "describe"
assert definition["stop"]["rpc"] == "Shutdown"
assert set(definition["stop"]) <= {"graceMs", "rpc"}
args = definition["command"]["args"]
assert args[args.index("--provider") + 1] == provider
PYTHON
    # scout-mini, mecanum-ugv, px4-multirotor: the profile files of the type
    profile_name="${provider#xgc2-}"
    profile_name="${profile_name%-ros1-adapter}"
    python3 "${REPO_ROOT}/tools/verify_profile_catalog.py" \
      --registry "${PROTOBUF_REGISTRY}" \
      --profile-file "${share}/profiles/ros1/${profile_name}-physical-vrpn.yaml" \
      --profile-file "${share}/profiles/ros1/${profile_name}-xsim-ros.yaml" \
      --profile-file "${share}/profiles/ros1/${profile_name}-gazebo-vrpn.yaml" \
      --profile-schema "${share}/profiles/schema/robot-adapter-profile-v4.schema.json" \
      --definition-id "${provider}" \
      --catalog "/usr/share/xgc2/robot-adapter-profiles/${provider}.json"
  done
}

check_type_server "${PX4_ROS_PACKAGE}" "${PX4_PACKAGE}" \
  xgc2-px4-multirotor-ros1-adapter
check_type_server "${UGV_ROS_PACKAGE}" "${UGV_PACKAGE}" \
  xgc2-scout-mini-ros1-adapter xgc2-mecanum-ugv-ros1-adapter

# Nothing of the structure before Run-owned type servers is left behind.
for removed in xgc_ros1_adapter xgc_scout_mini_ros1_adapter xgc_mecanum_ugv_ros1_adapter; do
  test ! -e "${PREFIX}/share/${removed}"
  test ! -e "${PREFIX}/lib/${removed}"
done
test ! -e /usr/share/xgc2/adapter-definitions

echo "Installed robot package checks passed"

for name in node service_helper probe clock_wait rosbag_recorder; do
  path="${PREFIX}/lib/${TOOLS_ROS_PACKAGE}/${TOOLS_ROS_PACKAGE}_${name}"
  test -x "${path}"
  dpkg-query -S "${path}" | grep -Fq "${TOOLS_PACKAGE}:"
done
tools_libraries="$(ldd "${PREFIX}/lib/${TOOLS_ROS_PACKAGE}/${TOOLS_ROS_PACKAGE}_node")"
if grep -q 'not found' <<<"${tools_libraries}"; then
  echo "the Tools process has unresolved libraries" >&2
  exit 1
fi
grep -q 'libxgc2_xrpc' <<<"${tools_libraries}"
python3 - <<'PYTOOLS'
import json
with open('/usr/share/xgc2/process-definitions/xgc2-ros1-tools-adapter.json') as source:
    document = json.load(source)
assert document['apiVersion'] == 'xgc.execution.process/v1'
definition = document['definitions'][0]
assert set(definition) <= {'id', 'version', 'label', 'description', 'parameters', 'setupFiles', 'command', 'services', 'readiness', 'stop', 'logs'}
assert definition['id'] == 'xgc2-ros1-tools-adapter'
assert definition['services'][0]['service'] == 'ros1.tools'
assert definition['services'][0]['profile'] == 'http.v1'
assert set(definition['services'][0]) == {'service', 'api_version', 'profile', 'endpointParameter'}
assert set(definition['parameters']) <= {'properties', 'required', 'groups'}
for parameter in definition['parameters']['properties'].values():
    assert set(parameter) <= {'type', 'default', 'enum', 'minimum', 'maximum', 'sensitive', 'output', 'fixedOnly'}
endpoint = definition['services'][0]['endpointParameter']
assert definition['parameters']['properties'][endpoint] == {'type': 'string', 'fixedOnly': True}
assert definition['readiness']['kind'] == 'describe'
assert definition['stop'] == {'graceMs': 5000}
PYTOOLS
echo "Installed Tools package checks passed"

scene_depends="$(dpkg-query -W -f='${Depends}' "${SCENE_PACKAGE}")"
grep -Fq 'libxgc2-xrpc2 (>= 0.2.0-1~focal)' <<<"${scene_depends}"
grep -Fq 'libxgc2-math-dev (>= 0.6.0-6~focal)' <<<"${scene_depends}"
grep -Fq "ros-${ROS_DISTRO}-xgc2-geometry-msgs (>= 1.5.0-4)" <<<"${scene_depends}"
for dependency in python3 roscpp tf2-ros geometry-msgs visualization-msgs std-msgs xgc2-geometry-msgs; do
  if [[ "${dependency}" == "python3" ]]; then
    grep -Eq '(^|, )python3( |[(,]|$)' <<<"${scene_depends}"
  else
    grep -Fq "ros-${ROS_DISTRO}-${dependency}" <<<"${scene_depends}"
  fi
done
if grep -Eq 'xgc2-protobuf-dev|libxgc2-xrpc-(grpc-)?dev' <<<"${scene_depends}"; then
  echo "scene adapter package depends on a build-only XRPC or protobuf package" >&2
  exit 1
fi
for dependency in nlohmann-json3-dev libyaml-cpp-dev libssl-dev libassimp-dev libpugixml-dev libsdformat9-dev pkg-config; do
  grep -Eq "(^|, )${dependency}( |[(,]|$)" <<<"${scene_depends}"
done
rospack find "${SCENE_ROS_PACKAGE}" >/dev/null
test -f "${PREFIX}/share/${SCENE_ROS_PACKAGE}/package.xml"
for name in xgc2_scene_adapter xgc2-scene-assemble xgc2-scene-prepare; do
  path="${PREFIX}/lib/${SCENE_ROS_PACKAGE}/${name}"
  test -x "${path}"
  dpkg-query -S "${path}" | grep -Fq "${SCENE_PACKAGE}:"
  libraries="$(ldd "${path}")"
  if grep -q 'not found' <<<"${libraries}"; then
    echo "${path} has unresolved libraries" >&2
    exit 1
  fi
  if [[ "${name}" == "xgc2_scene_adapter" ]]; then
    grep -q 'libxgc2_xrpc' <<<"${libraries}"
  fi
done
"${PREFIX}/lib/${SCENE_ROS_PACKAGE}/xgc2-scene-prepare" --help >/dev/null
scene_definition="/usr/share/xgc2/process-definitions/xgc2-ros1-scene-adapter.json"
dpkg-query -S "${scene_definition}" | grep -Fq "${SCENE_PACKAGE}:"
cmp "${scene_definition}" "${REPO_ROOT}/src/scene_ros1_adapter/process/xgc2-ros1-scene-adapter.json"
python3 - "${scene_definition}" "${PREFIX}/lib/${SCENE_ROS_PACKAGE}/xgc2_scene_adapter" <<'PYSCENE'
import json
import sys

path, executable = sys.argv[1:]
with open(path) as source:
    document = json.load(source)
assert document['apiVersion'] == 'xgc.execution.process/v1'
(definition,) = document['definitions']
assert set(definition) <= {'id', 'version', 'label', 'description', 'parameters', 'setupFiles', 'command', 'services', 'readiness', 'stop', 'logs'}
assert definition['id'] == 'xgc2-ros1-scene-adapter'
assert definition['command']['executable'] == executable
assert definition['services'] == [
    {'service': 'xgc2.scene.authoring', 'api_version': 'v1', 'profile': 'http.v1', 'endpointParameter': 'socketPath'},
    {'service': 'xgc2.sensor.lidar', 'api_version': 'v1', 'profile': 'http.v1', 'endpointParameter': 'lidarSocketPath'},
]
assert set(definition['parameters']) <= {'properties', 'required', 'groups'}
for parameter in definition['parameters']['properties'].values():
    assert set(parameter) <= {'type', 'description', 'default', 'enum', 'minimum', 'maximum', 'sensitive', 'output', 'fixedOnly'}
assert definition['parameters']['properties']['socketPath'] == {'type': 'string', 'fixedOnly': True}
assert definition['parameters']['properties']['lidarSocketPath'] == {'type': 'string', 'fixedOnly': True}
assert definition['parameters']['properties']['targetId'] == {'type': 'string', 'default': 'local', 'fixedOnly': True}
endpoint = definition['parameters']['properties']['simulationEndpoint']
assert endpoint['type'] == 'string' and endpoint['fixedOnly'] and endpoint['default'] == ''
assert '$session.endpoints.world' in endpoint['description']
assert 'simulationServiceRefJson' not in definition['parameters']['properties']
assert '_simulation_endpoint:=${simulationEndpoint}' in definition['command']['args']
assert not any(arg.startswith('_simulation_service_ref_json:=') for arg in definition['command']['args'])
# The frozen session facts arrive on standard input (the lidar fleet follows the roster).
assert definition['command']['stdinParameter'] == 'sessionJson'
assert definition['parameters']['properties']['sessionJson']['type'] == 'string'
assert '--session-stdin' in definition['command']['args']
assert definition['parameters']['properties']['profile']['enum'] == ['gazebo', 'xsim', 'physical']
assert definition['readiness'] == {'kind': 'describe', 'startGraceMs': 15000, 'timeoutMs': 30000}
assert definition['stop'] == {'graceMs': 5000}
PYSCENE
# A fresh consumer must find only the installed target, including its link closure.
for path in "${PREFIX}/lib/libxgc_scene_core.a" \
    "${PREFIX}/lib/cmake/XgcSceneCore/"*.cmake \
    "${PREFIX}/include/xgc_scene/"*.hpp; do
  test -f "${path}"
  dpkg-query -S "${path}" | grep -Fq "${SCENE_PACKAGE}:"
done
scene_core_check_root="$(mktemp -d)"
trap 'rm -rf "${scene_core_check_root}"' EXIT
cmake -S "${REPO_ROOT}/src/scene_ros1_adapter/test/installed_core_consumer" \
  -B "${scene_core_check_root}" -DCMAKE_BUILD_TYPE=Release \
  "-DCMAKE_PREFIX_PATH=${PREFIX}"
cmake --build "${scene_core_check_root}" -j1
(cd "${scene_core_check_root}" && ./consumer)
rm -rf "${scene_core_check_root}"
trap - EXIT
echo "Installed Scene package checks passed"
