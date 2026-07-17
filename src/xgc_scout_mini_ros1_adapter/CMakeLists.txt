cmake_minimum_required(VERSION 3.0.2)
project(xgc_scout_mini_ros1_adapter VERSION 0.5.0)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  scout_msgs
  sensor_msgs
)
find_package(PythonInterp 3 REQUIRED)
find_package(xgc2_adapter_runtime_client CONFIG REQUIRED)

include("${CMAKE_CURRENT_SOURCE_DIR}/../../common/cmake/robot_adapter_common.cmake")
xgc2_add_ros1_robot_common(${PROJECT_NAME}_runtime_support)

set(PROFILE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../profiles")
set(PROFILE_FILE
  "${PROFILE_ROOT}/ros1/scout-mini-ros1-v3.yaml")
set(PROFILE_SCHEMA
  "${PROFILE_ROOT}/schema/robot-adapter-profile-v3.schema.json")
set(ADAPTER_DEFINITION_ID "xgc2-scout-mini-ros1-adapter")

if(NOT EXISTS "${XGC2_PROTOBUF_REGISTRY_JSON}")
  message(FATAL_ERROR "Adapter Runtime dependencies did not expose the XGC2 registry")
endif()
if(NOT EXISTS "${PROFILE_FILE}" OR NOT EXISTS "${PROFILE_SCHEMA}")
  message(FATAL_ERROR "repository-owned Scout Mini ROS1 profile contract is incomplete")
endif()

set(CONTRACT_METADATA_HDR
  "${CMAKE_CURRENT_BINARY_DIR}/generated/${PROJECT_NAME}/generated_contract.hpp")
set(CONTRACT_GENERATOR
  "${CMAKE_CURRENT_SOURCE_DIR}/../../tools/generate_contract_metadata.py")

add_custom_command(
  OUTPUT "${CONTRACT_METADATA_HDR}"
  COMMAND ${CMAKE_COMMAND} -E make_directory
    "${CMAKE_CURRENT_BINARY_DIR}/generated/${PROJECT_NAME}"
  COMMAND ${PYTHON_EXECUTABLE} "${CONTRACT_GENERATOR}"
    --registry "${XGC2_PROTOBUF_REGISTRY_JSON}"
    --profile-file "${PROFILE_FILE}"
    --profile-schema "${PROFILE_SCHEMA}"
    --definition-id "${ADAPTER_DEFINITION_ID}"
    --cpp-namespace "${PROJECT_NAME}"
    --output "${CONTRACT_METADATA_HDR}"
  DEPENDS
    "${CONTRACT_GENERATOR}"
    "${XGC2_PROTOBUF_REGISTRY_JSON}"
    "${PROFILE_FILE}"
    "${PROFILE_SCHEMA}"
  COMMENT "Validating the repository-owned Scout Mini ROS1 profile"
  VERBATIM
)
add_custom_target(${PROJECT_NAME}_contract_metadata
  DEPENDS "${CONTRACT_METADATA_HDR}")

catkin_package()

include_directories(
  include
  "${CMAKE_CURRENT_BINARY_DIR}/generated"
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/scout_mini_ros1_adapter_node.cpp
  src/robot_runtime.cpp
  src/shutdown_signal.cpp
  "${CONTRACT_METADATA_HDR}"
)
add_dependencies(${PROJECT_NAME}_node
  ${PROJECT_NAME}_contract_metadata
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_runtime_support
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  add_test(NAME ${PROJECT_NAME}_profile_metadata_test
    COMMAND ${PYTHON_EXECUTABLE}
      "${CMAKE_CURRENT_SOURCE_DIR}/../../test/test_generate_contract_metadata.py"
  )
  catkin_add_gtest(${PROJECT_NAME}_test
    test/robot_runtime_test.cpp
    src/robot_runtime.cpp
    src/shutdown_signal.cpp
  )
  if(TARGET ${PROJECT_NAME}_test)
    add_dependencies(${PROJECT_NAME}_test
      ${PROJECT_NAME}_contract_metadata
      ${catkin_EXPORTED_TARGETS}
    )
    target_link_libraries(${PROJECT_NAME}_test
      ${PROJECT_NAME}_runtime_support
      ${catkin_LIBRARIES}
    )
  endif()
endif()

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
xgc2_add_robot_runtime_manifests(
  ${PROJECT_NAME}_node
  ${ADAPTER_DEFINITION_ID}
  ${PROJECT_VERSION}
  "XGC2 Scout Mini ROS1 Adapter"
  "Scout Mini telemetry and channel-diagnostic capabilities."
  "${PROFILE_FILE}"
  "${PROFILE_SCHEMA}"
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(FILES "${PROFILE_FILE}"
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/profiles/ros1
)
install(FILES "${PROFILE_SCHEMA}"
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/profiles/schema
)
