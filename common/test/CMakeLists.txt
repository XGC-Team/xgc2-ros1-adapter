# These tools exercise real TCPROS and installed servers. They never ship in a
# robot package and add no production thread or instrumentation dependency.
add_executable(async_ros_services_driver EXCLUDE_FROM_ALL async_ros_services_driver.cpp)
target_link_libraries(async_ros_services_driver ${PROJECT_NAME}_runtime_support)
foreach(tool ros_forwarding_source ros_forwarding_observer)
  add_executable(${tool} EXCLUDE_FROM_ALL ${tool}.cpp)
  target_include_directories(${tool} PRIVATE ${catkin_INCLUDE_DIRS})
  target_link_libraries(${tool} ${catkin_LIBRARIES})
endforeach()
add_library(robot_server_allocation_probe SHARED EXCLUDE_FROM_ALL
  allocation_probe.c ros_publish_probe.cpp)
target_include_directories(robot_server_allocation_probe PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(robot_server_allocation_probe ${CMAKE_DL_LIBS})
set_target_properties(async_ros_services_driver ros_forwarding_source
  ros_forwarding_observer PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/lib/robot_server_tests")
set_target_properties(robot_server_allocation_probe PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/lib/robot_server_tests")
add_custom_target(robot_server_test_tools DEPENDS async_ros_services_driver
  ros_forwarding_source ros_forwarding_observer robot_server_allocation_probe)
catkin_add_gtest(robot_server_entry_test robot_server_entry_test.cpp)
if(TARGET robot_server_entry_test)
  target_include_directories(robot_server_entry_test PRIVATE "${XGC2_ROS1_ROBOT_COMMON_ROOT}/include")
endif()
