catkin_add_gtest(${PROJECT_NAME}_options_test robot_server_options_test.cpp)
if(TARGET ${PROJECT_NAME}_options_test)
  target_link_libraries(${PROJECT_NAME}_options_test xgc2_robot_server)
endif()

# The server core against a fake RosBinding, over real gRPC on a Unix socket.
catkin_add_gtest(${PROJECT_NAME}_server_test robot_server_test.cpp)
if(TARGET ${PROJECT_NAME}_server_test)
  target_include_directories(${PROJECT_NAME}_server_test PRIVATE ${GRPC_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME}_server_test
    xgc2_robot_server XgcXrpc::grpc nlohmann_json::nlohmann_json ${GRPC_LDFLAGS})
endif()

catkin_add_gtest(${PROJECT_NAME}_domain_test robot_domain_test.cpp)
if(TARGET ${PROJECT_NAME}_domain_test)
  target_link_libraries(${PROJECT_NAME}_domain_test
    xgc2_robot_server_ros1 protobuf::libprotobuf)
endif()

# Validation driver of the TCPROS service client; test_async_ros_services.py
# runs it against a controlled transport fixture.
add_executable(async_ros_services_driver EXCLUDE_FROM_ALL
  async_ros_services_driver.cpp)
target_link_libraries(async_ros_services_driver xgc2_robot_server_ros1)
set_target_properties(async_ros_services_driver PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}")

# Command-line gRPC client of the type servers: the integration tests run it
# because the machines that build these servers have no Python gRPC.
add_executable(robot_server_client EXCLUDE_FROM_ALL robot_server_client.cpp)
target_include_directories(robot_server_client PRIVATE ${GRPC_INCLUDE_DIRS})
target_link_libraries(robot_server_client
  xgc2_robot_server XgcXrpc::grpc nlohmann_json::nlohmann_json ${GRPC_LDFLAGS})
set_target_properties(robot_server_client PROPERTIES
  RUNTIME_OUTPUT_DIRECTORY "${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME}")
