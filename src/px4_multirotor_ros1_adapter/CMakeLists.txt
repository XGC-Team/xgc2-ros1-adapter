cmake_minimum_required(VERSION 3.16)
project(xgc_px4_multirotor_ros1_adapter VERSION 0.6.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  sensor_msgs
  xgc_ros1_adapter_common
)
find_package(Protobuf REQUIRED)
if(NOT XGC2_PROTOBUF_REGISTRY_JSON)
  find_package(xgc2_protobuf CONFIG REQUIRED)
endif()

include("${CMAKE_CURRENT_SOURCE_DIR}/../common/cmake/robot_profiles.cmake")
set(CONTRACT_HEADER
  "${CMAKE_CURRENT_BINARY_DIR}/generated/${PROJECT_NAME}/generated_contract.hpp")
xgc2_add_robot_profiles(
  TARGET ${PROJECT_NAME}_profiles
  DEFINITION_ID xgc2-px4-multirotor-ros1-adapter
  NAMESPACE ${PROJECT_NAME}
  HEADER "${CONTRACT_HEADER}"
  PROFILES
    "${XGC2_ROBOT_PROFILE_ROOT}/ros1/px4-multirotor-physical-vrpn.yaml"
    "${XGC2_ROBOT_PROFILE_ROOT}/ros1/px4-multirotor-xsim-ros.yaml"
    "${XGC2_ROBOT_PROFILE_ROOT}/ros1/px4-multirotor-gazebo-vrpn.yaml"
)

catkin_package(
  CATKIN_DEPENDS mavros_msgs roscpp xgc_ros1_adapter_common
)

include_directories(
  include
  "${CMAKE_CURRENT_BINARY_DIR}/generated"
  ${catkin_INCLUDE_DIRS}
)

# PX4 service requests, MAVROS interpretation and the remote-control stream.
add_library(${PROJECT_NAME}_operations STATIC
  src/px4_operations.cpp
  src/remote_control.cpp
)
add_dependencies(${PROJECT_NAME}_operations ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_operations ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node
  src/px4_multirotor_ros1_adapter_node.cpp
  src/robot_runtime.cpp
  "${CONTRACT_HEADER}"
)
add_dependencies(${PROJECT_NAME}_node
  ${PROJECT_NAME}_profiles
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_operations
  ${catkin_LIBRARIES}
  protobuf::libprotobuf
)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES process/xgc2-px4-multirotor-ros1-adapter.json
  DESTINATION /usr/share/xgc2/process-definitions
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(${PROJECT_NAME}_status_demand_test
    test/status_demand.test test/status_demand_test.cpp src/robot_runtime.cpp)
  if(TARGET ${PROJECT_NAME}_status_demand_test)
    add_dependencies(${PROJECT_NAME}_status_demand_test ${PROJECT_NAME}_profiles ${catkin_EXPORTED_TARGETS})
    target_link_libraries(${PROJECT_NAME}_status_demand_test ${catkin_LIBRARIES} protobuf::libprotobuf)
  endif()
  catkin_add_gtest(${PROJECT_NAME}_test
    test/robot_runtime_test.cpp
    src/robot_runtime.cpp
  )
  if(TARGET ${PROJECT_NAME}_test)
    add_dependencies(${PROJECT_NAME}_test
      ${PROJECT_NAME}_profiles
      ${catkin_EXPORTED_TARGETS}
    )
    target_link_libraries(${PROJECT_NAME}_test
      ${catkin_LIBRARIES}
      protobuf::libprotobuf
    )
  endif()
  catkin_add_gtest(${PROJECT_NAME}_operations_test
    test/px4_operations_test.cpp
    test/remote_control_test.cpp
  )
  if(TARGET ${PROJECT_NAME}_operations_test)
    add_dependencies(${PROJECT_NAME}_operations_test ${catkin_EXPORTED_TARGETS})
    target_link_libraries(${PROJECT_NAME}_operations_test
      ${PROJECT_NAME}_operations
      ${catkin_LIBRARIES}
    )
  endif()
endif()
