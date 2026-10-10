cmake_minimum_required(VERSION 3.16)
project(xgc_ugv_ros1_adapter VERSION 0.6.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  xgc_ros1_adapter_common
)
find_package(Protobuf REQUIRED)
if(NOT XGC2_PROTOBUF_REGISTRY_JSON)
  find_package(xgc2_protobuf CONFIG REQUIRED)
endif()

# One runtime serves every ground vehicle type; the contract of each type is
# generated from its own profiles.
include("${CMAKE_CURRENT_SOURCE_DIR}/../common/cmake/robot_profiles.cmake")
set(GENERATED_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated/${PROJECT_NAME}")
xgc2_add_robot_profiles(
  TARGET ${PROJECT_NAME}_scout_mini_profiles
  DEFINITION_ID xgc2-scout-mini-ros1-adapter
  NAMESPACE ${PROJECT_NAME}::scout_mini
  HEADER "${GENERATED_DIR}/scout_mini_contract.hpp"
  PROFILES
    "${XGC2_ROBOT_PROFILE_ROOT}/ros1/scout-mini-physical-vrpn.yaml"
    "${XGC2_ROBOT_PROFILE_ROOT}/ros1/scout-mini-xsim-ros.yaml"
    "${XGC2_ROBOT_PROFILE_ROOT}/ros1/scout-mini-gazebo-vrpn.yaml"
)
xgc2_add_robot_profiles(
  TARGET ${PROJECT_NAME}_mecanum_ugv_profiles
  DEFINITION_ID xgc2-mecanum-ugv-ros1-adapter
  NAMESPACE ${PROJECT_NAME}::mecanum_ugv
  HEADER "${GENERATED_DIR}/mecanum_ugv_contract.hpp"
  PROFILES
    "${XGC2_ROBOT_PROFILE_ROOT}/ros1/mecanum-ugv-physical-vrpn.yaml"
    "${XGC2_ROBOT_PROFILE_ROOT}/ros1/mecanum-ugv-xsim-ros.yaml"
    "${XGC2_ROBOT_PROFILE_ROOT}/ros1/mecanum-ugv-gazebo-vrpn.yaml"
)

catkin_package(
  CATKIN_DEPENDS roscpp xgc_ros1_adapter_common
)

include_directories(
  include
  "${CMAKE_CURRENT_BINARY_DIR}/generated"
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_runtime STATIC
  src/motion_command.cpp
  src/robot_runtime.cpp
  src/variant.cpp
)
add_dependencies(${PROJECT_NAME}_runtime
  ${PROJECT_NAME}_scout_mini_profiles
  ${PROJECT_NAME}_mecanum_ugv_profiles
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_runtime
  ${catkin_LIBRARIES}
  protobuf::libprotobuf
)

add_executable(${PROJECT_NAME}_node src/ugv_ros1_adapter_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_runtime)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY process/
  DESTINATION /usr/share/xgc2/process-definitions
  FILES_MATCHING PATTERN "*.json"
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}_test test/robot_runtime_test.cpp)
  if(TARGET ${PROJECT_NAME}_test)
    target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME}_runtime)
  endif()
endif()
