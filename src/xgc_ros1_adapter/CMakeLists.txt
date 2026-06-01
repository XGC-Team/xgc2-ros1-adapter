cmake_minimum_required(VERSION 3.0.2)
project(xgc_ros1_adapter)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  sensor_msgs
  std_msgs
)
find_package(Protobuf REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GRPCPP REQUIRED grpc++)

if(DEFINED ENV{XGC2_CONTRACTS_DIR})
  set(XGC2_CONTRACTS_DIR "$ENV{XGC2_CONTRACTS_DIR}" CACHE PATH "XGC2 contracts directory")
else()
  set(_xgc2_contract_candidates
    "${CMAKE_CURRENT_SOURCE_DIR}/../../contracts"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../../../../temp/xgc2/contracts"
  )
  foreach(_candidate ${_xgc2_contract_candidates})
    get_filename_component(_candidate_abs "${_candidate}" ABSOLUTE)
    if(EXISTS "${_candidate_abs}/adapter/v1/adapter.proto")
      set(XGC2_CONTRACTS_DIR "${_candidate_abs}" CACHE PATH "XGC2 contracts directory")
      break()
    endif()
  endforeach()
endif()

set(ADAPTER_PROTO "${XGC2_CONTRACTS_DIR}/adapter/v1/adapter.proto")
if(NOT EXISTS "${ADAPTER_PROTO}")
  message(FATAL_ERROR "adapter.proto not found. Set XGC2_CONTRACTS_DIR to the XGC2 contracts directory or place contracts under this workspace root.")
endif()

set(GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(GENERATED_PROTO_DIR "${GENERATED_DIR}/adapter/v1")
set(GENERATED_SRCS
  "${GENERATED_PROTO_DIR}/adapter.pb.cc"
  "${GENERATED_PROTO_DIR}/adapter.grpc.pb.cc"
)
set(GENERATED_HDRS
  "${GENERATED_PROTO_DIR}/adapter.pb.h"
  "${GENERATED_PROTO_DIR}/adapter.grpc.pb.h"
)

add_custom_command(
  OUTPUT ${GENERATED_SRCS} ${GENERATED_HDRS}
  COMMAND ${CMAKE_COMMAND} -E make_directory "${GENERATED_DIR}"
  COMMAND /usr/bin/protoc
    --proto_path=${XGC2_CONTRACTS_DIR}
    --cpp_out=${GENERATED_DIR}
    --grpc_out=${GENERATED_DIR}
    --plugin=protoc-gen-grpc=/usr/bin/grpc_cpp_plugin
    ${ADAPTER_PROTO}
  DEPENDS "${ADAPTER_PROTO}"
  COMMENT "Generating XGC adapter protobuf and gRPC C++ bindings"
  VERBATIM
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GENERATED_DIR}
  ${Protobuf_INCLUDE_DIRS}
  ${GRPCPP_INCLUDE_DIRS}
)

add_library(xgc_adapter_proto ${GENERATED_SRCS})
target_link_libraries(xgc_adapter_proto
  ${Protobuf_LIBRARIES}
  ${GRPCPP_LIBRARIES}
)

add_executable(xgc_ros1_adapter_node
  src/ros1_adapter_node.cpp
  src/robot_runtime.cpp
  src/robot_behavior.cpp
  src/ugv_behavior.cpp
  src/tello_behavior.cpp
  src/px4_rotor_behavior.cpp
  src/fw_plane_behavior.cpp
)
add_dependencies(xgc_ros1_adapter_node xgc_adapter_proto)
target_link_libraries(xgc_ros1_adapter_node
  xgc_adapter_proto
  ${catkin_LIBRARIES}
  ${Protobuf_LIBRARIES}
  ${GRPCPP_LIBRARIES}
)
