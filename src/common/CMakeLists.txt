cmake_minimum_required(VERSION 3.16)
project(xgc_ros1_adapter_common VERSION 0.6.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS geometry_msgs roscpp)
find_package(Threads REQUIRED)
find_package(Protobuf REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GRPC REQUIRED grpc++>=1.16 grpc)
find_package(XgcXrpc 0.1 REQUIRED CONFIG COMPONENTS grpc)
find_package(nlohmann_json 3.7 REQUIRED)
if(NOT XGC2_PROTOBUF_PROTO_ROOT)
  find_package(xgc2_protobuf CONFIG REQUIRED)
endif()
find_program(GRPC_CPP_PLUGIN grpc_cpp_plugin)
if(NOT GRPC_CPP_PLUGIN)
  message(FATAL_ERROR "grpc_cpp_plugin (protobuf-compiler-grpc) is required")
endif()
# c-ares comes from the distribution package or from any prefix on
# CMAKE_PREFIX_PATH; pkg-config cannot express a relocated development root.
find_path(CARES_INCLUDE_DIR ares.h)
find_library(CARES_LIBRARY cares)
if(NOT CARES_INCLUDE_DIR OR NOT CARES_LIBRARY)
  message(FATAL_ERROR "c-ares development files (ares.h, libcares) are required")
endif()

# ---- the robot protocol, generated once for every type server ---------------
if(NOT EXISTS "${XGC2_PROTOBUF_PROTO_ROOT}/xgc/robot/v1/server.proto")
  message(FATAL_ERROR "xgc2-protobuf-dev does not provide the robot type server protocol")
endif()
set(protocol_files
  xgc/v1/message.proto
  xgc/adapter/v1/adapter.proto
  xgc/robot/v1/message.proto
  xgc/robot/v1/server.proto
  xgc/semantic/aerial/v1/control.proto
  xgc/semantic/aerial/v1/diagnostic.proto
  xgc/semantic/aerial/v1/flight.proto
  xgc/semantic/aerial/v1/setpoint.proto
  xgc/semantic/common/v1/acceleration.proto
  xgc/semantic/common/v1/control.proto
  xgc/semantic/common/v1/telemetry.proto
  xgc/semantic/common/v1/types.proto
  xgc/semantic/ground/v1/chassis.proto
  xgc/semantic/ground/v1/control.proto
  xgc/semantic/ground/v1/locomotion.proto
)
set(proto_dir "${CMAKE_CURRENT_BINARY_DIR}/proto")
set(protocol_inputs)
set(protocol_sources)
set(protocol_headers)
foreach(relative_file IN LISTS protocol_files)
  list(APPEND protocol_inputs "${XGC2_PROTOBUF_PROTO_ROOT}/${relative_file}")
  string(REGEX REPLACE "\\.proto$" ".pb.cc" generated_source "${relative_file}")
  string(REGEX REPLACE "\\.proto$" ".pb.h" generated_header "${relative_file}")
  list(APPEND protocol_sources "${proto_dir}/${generated_source}")
  list(APPEND protocol_headers "${proto_dir}/${generated_header}")
endforeach()
set(grpc_source "${proto_dir}/xgc/robot/v1/server.grpc.pb.cc")
set(grpc_header "${proto_dir}/xgc/robot/v1/server.grpc.pb.h")
add_custom_command(
  OUTPUT ${protocol_sources} ${protocol_headers} "${grpc_source}" "${grpc_header}"
  COMMAND "${CMAKE_COMMAND}" -E make_directory "${proto_dir}"
  COMMAND "${Protobuf_PROTOC_EXECUTABLE}"
          "--proto_path=${XGC2_PROTOBUF_PROTO_ROOT}" "--cpp_out=${proto_dir}"
          ${protocol_inputs}
  COMMAND "${Protobuf_PROTOC_EXECUTABLE}"
          "--proto_path=${XGC2_PROTOBUF_PROTO_ROOT}" "--grpc_out=${proto_dir}"
          "--plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN}"
          "${XGC2_PROTOBUF_PROTO_ROOT}/xgc/robot/v1/server.proto"
  DEPENDS ${protocol_inputs}
  COMMENT "Generating the robot type server protocol"
  VERBATIM
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES xgc2_robot_server xgc2_robot_server_ros1
  CATKIN_DEPENDS geometry_msgs roscpp
  CFG_EXTRAS xgc_ros1_adapter_common-extras.cmake
)

# ---- ROS-agnostic core: server, members, profiles, protocol -----------------
add_library(xgc2_robot_server SHARED
  src/contract.cpp
  src/ground_health.cpp
  src/robot_domain.cpp
  src/robot_server.cpp
  src/robot_server_options.cpp
  ${protocol_sources}
  "${grpc_source}"
)
set_target_properties(xgc2_robot_server PROPERTIES VERSION ${PROJECT_VERSION}
                      SOVERSION 0)
# The generated code is the pinned protocol, not ours to lint: its directory is
# a system include.
target_include_directories(xgc2_robot_server
  PUBLIC include
  PRIVATE ${GRPC_INCLUDE_DIRS})
target_include_directories(xgc2_robot_server SYSTEM PUBLIC "${proto_dir}")
target_link_libraries(xgc2_robot_server
  PUBLIC protobuf::libprotobuf Threads::Threads
  PRIVATE XgcXrpc::grpc nlohmann_json::nlohmann_json ${GRPC_LDFLAGS})
set_source_files_properties(${protocol_sources} "${grpc_source}" PROPERTIES
  COMPILE_FLAGS "-Wno-array-bounds")

# ---- ROS1 binding: the only code that includes roscpp ------------------------
# This library defines ros::master::execute before libroscpp in every type
# server's link order; see src/bounded_ros_master.cpp.
add_library(xgc2_robot_server_ros1 SHARED
  src/async_ros_services.cpp
  src/bounded_ros_master.cpp
  src/localization_projection.cpp
  src/ros1_binding.cpp
  src/ros1_run.cpp
)
set_target_properties(xgc2_robot_server_ros1 PROPERTIES VERSION ${PROJECT_VERSION}
                      SOVERSION 0)
target_include_directories(xgc2_robot_server_ros1
  PUBLIC include ${catkin_INCLUDE_DIRS}
  PRIVATE "${CARES_INCLUDE_DIR}")
target_include_directories(xgc2_robot_server_ros1 SYSTEM PUBLIC "${proto_dir}")
target_link_libraries(xgc2_robot_server_ros1
  PUBLIC xgc2_robot_server ${catkin_LIBRARIES}
  PRIVATE "${CARES_LIBRARY}")

install(TARGETS xgc2_robot_server xgc2_robot_server_ros1
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp")
install(DIRECTORY "${proto_dir}/"
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/xgc_ros1_adapter_common/proto
  FILES_MATCHING PATTERN "*.h")

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()
