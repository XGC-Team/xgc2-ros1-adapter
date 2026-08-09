cmake_minimum_required(VERSION 3.0.2)
project(xgc_mocap_rotor_zenoh_forwarder VERSION 0.5.0)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)
find_package(catkin REQUIRED COMPONENTS geometry_msgs mavros_msgs roscpp sensor_msgs)
find_package(zenohc CONFIG REQUIRED)

catkin_package()
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}_core STATIC
  src/forwarder_contract.cpp
  src/wire_encoder.cpp
  src/zenoh_publisher.cpp)
add_dependencies(${PROJECT_NAME}_core ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_core ${catkin_LIBRARIES} zenohc::static)

add_executable(${PROJECT_NAME}_node src/mocap_rotor_zenoh_forwarder_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_core ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}_contract_test test/forwarder_contract_test.cpp)
  if(TARGET ${PROJECT_NAME}_contract_test)
    target_link_libraries(${PROJECT_NAME}_contract_test ${PROJECT_NAME}_core)
  endif()
  catkin_add_gtest(${PROJECT_NAME}_wire_encoder_test test/wire_encoder_test.cpp)
  if(TARGET ${PROJECT_NAME}_wire_encoder_test)
    target_link_libraries(${PROJECT_NAME}_wire_encoder_test ${PROJECT_NAME}_core)
  endif()
  catkin_add_gtest(${PROJECT_NAME}_zenoh_loopback_test test/zenoh_loopback_test.cpp)
  if(TARGET ${PROJECT_NAME}_zenoh_loopback_test)
    target_link_libraries(${PROJECT_NAME}_zenoh_loopback_test
      ${PROJECT_NAME}_core zenohc::static)
  endif()
endif()

install(TARGETS ${PROJECT_NAME}_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
install(FILES process/xgc2-mocap-rotor-link.json
  DESTINATION /usr/share/xgc2/process-definitions)
