cmake_minimum_required(VERSION 3.0.2)
project(xgc_px4_multirotor_ros1_adapter)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  sensor_msgs
)
find_package(PythonInterp 3 REQUIRED)
find_package(xgc2_adapter_link_client CONFIG REQUIRED)

if(NOT EXISTS "${XGC2_PROTOBUF_REGISTRY_JSON}")
  message(FATAL_ERROR "AdapterLink client dependency did not expose the XGC2 registry")
endif()
if(NOT EXISTS "${XGC2_PROTOBUF_PROFILES_DIR}/ros1/px4-multirotor-ros1-v2.yaml")
  message(FATAL_ERROR "installed XGC2 contract does not provide the PX4 ROS1 profile")
endif()

set(CONTRACT_METADATA_HDR
  "${CMAKE_CURRENT_BINARY_DIR}/generated/${PROJECT_NAME}/generated_contract.hpp")
set(CONTRACT_GENERATOR
  "${CMAKE_CURRENT_SOURCE_DIR}/../../tools/generate_contract_metadata.py")
set(PROFILE_FILE
  "${XGC2_PROTOBUF_PROFILES_DIR}/ros1/px4-multirotor-ros1-v2.yaml")

add_custom_command(
  OUTPUT "${CONTRACT_METADATA_HDR}"
  COMMAND ${CMAKE_COMMAND} -E make_directory
    "${CMAKE_CURRENT_BINARY_DIR}/generated/${PROJECT_NAME}"
  COMMAND ${PYTHON_EXECUTABLE} "${CONTRACT_GENERATOR}"
    --registry "${XGC2_PROTOBUF_REGISTRY_JSON}"
    --profiles-dir "${XGC2_PROTOBUF_PROFILES_DIR}"
    --profile-id "px4.multirotor.ros1.v2"
    --cpp-namespace "${PROJECT_NAME}"
    --output "${CONTRACT_METADATA_HDR}"
  DEPENDS
    "${CONTRACT_GENERATOR}"
    "${XGC2_PROTOBUF_REGISTRY_JSON}"
    "${PROFILE_FILE}"
  COMMENT "Validating the installed PX4 multirotor ROS1 profile"
  VERBATIM
)
add_custom_target(${PROJECT_NAME}_contract_metadata
  DEPENDS "${CONTRACT_METADATA_HDR}")

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp sensor_msgs
)

include_directories(
  include
  "${CMAKE_CURRENT_BINARY_DIR}/generated"
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node
  src/px4_multirotor_ros1_adapter_node.cpp
  src/robot_runtime.cpp
  src/shutdown_signal.cpp
  "${CONTRACT_METADATA_HDR}"
)
add_dependencies(${PROJECT_NAME}_node
  ${PROJECT_NAME}_contract_metadata
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_node
  xgc2::adapter_link_client
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
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
      xgc2::adapter_link_client
      ${catkin_LIBRARIES}
    )
  endif()
endif()

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
