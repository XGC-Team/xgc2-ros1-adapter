cmake_minimum_required(VERSION 3.10)
project(xgc_ros1_tools_adapter VERSION 0.5.0 LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS
  ros_babel_fish
  roscpp
  roslib
  rosgraph_msgs
)
find_package(PkgConfig REQUIRED)
find_package(PythonInterp 3 REQUIRED)
find_package(Threads REQUIRED)
find_package(XgcXrpc 0.1 REQUIRED CONFIG)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)

catkin_package(
  CATKIN_DEPENDS ros_babel_fish roscpp roslib rosgraph_msgs
)

add_library(${PROJECT_NAME}_core STATIC
  src/json_codec.cpp
  src/publisher_registry.cpp
  src/service_invoker.cpp
  src/tools.cpp
  src/type_registry.cpp
)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME}_core PRIVATE
  -Wall -Wextra -Wpedantic -Werror
)
target_include_directories(${PROJECT_NAME}_core
  PUBLIC
    include
    ${catkin_INCLUDE_DIRS}
    ${JSONCPP_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_core
  PUBLIC
    ${catkin_LIBRARIES}
    ${JSONCPP_LIBRARIES}
    Threads::Threads
)

add_executable(${PROJECT_NAME}_node src/main.cpp)
target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_20)
target_compile_options(${PROJECT_NAME}_node PRIVATE
  -Wall -Wextra -Wpedantic -Werror
)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME}_core XgcXrpc::http)

add_executable(${PROJECT_NAME}_service_helper src/service_helper_main.cpp)
target_compile_features(${PROJECT_NAME}_service_helper PUBLIC cxx_std_17)
target_compile_options(${PROJECT_NAME}_service_helper PRIVATE
  -Wall -Wextra -Wpedantic -Werror
)
target_include_directories(${PROJECT_NAME}_service_helper PRIVATE
  include
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_service_helper
  ${catkin_LIBRARIES}
  Threads::Threads
)

add_executable(${PROJECT_NAME}_clock_wait src/clock_wait_main.cpp)
target_compile_features(${PROJECT_NAME}_clock_wait PRIVATE cxx_std_14)
target_compile_options(${PROJECT_NAME}_clock_wait PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_include_directories(${PROJECT_NAME}_clock_wait PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_clock_wait ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_probe src/probe_main.cpp)
target_compile_features(${PROJECT_NAME}_probe PRIVATE cxx_std_17)
target_compile_options(${PROJECT_NAME}_probe PRIVATE -Wall -Wextra -Wpedantic -Werror)
target_include_directories(${PROJECT_NAME}_probe PRIVATE ${catkin_INCLUDE_DIRS} ${JSONCPP_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_probe ${catkin_LIBRARIES} ${JSONCPP_LIBRARIES})

install(TARGETS
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}_service_helper
  ${PROJECT_NAME}_clock_wait
  ${PROJECT_NAME}_probe
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS "${CMAKE_CURRENT_SOURCE_DIR}/scripts/rosbag_recorder.py"
 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 RENAME ${PROJECT_NAME}_rosbag_recorder)

install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/schemas/"
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/schemas
)
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/process/xgc2-ros1-tools-adapter.json"
  DESTINATION "/usr/share/xgc2/process-definitions")

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(std_srvs REQUIRED)

  catkin_add_gtest(${PROJECT_NAME}_unit_tests
    test/json_codec_test.cpp
    test/service_helper_protocol_test.cpp
    test/type_catalog_test.cpp
    test/clock_wait_test.cpp
  )
  if(TARGET ${PROJECT_NAME}_unit_tests)
    target_compile_features(${PROJECT_NAME}_unit_tests PUBLIC cxx_std_17)
    target_compile_options(${PROJECT_NAME}_unit_tests PRIVATE
      -Wall -Wextra -Wpedantic -Werror
    )
    target_include_directories(${PROJECT_NAME}_unit_tests PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${JSONCPP_INCLUDE_DIRS}
    )
    target_link_libraries(${PROJECT_NAME}_unit_tests
      ${PROJECT_NAME}_core
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(${PROJECT_NAME}_service_helper_lifecycle_tests
    test/service_helper_lifecycle_test.cpp
  )
  if(TARGET ${PROJECT_NAME}_service_helper_lifecycle_tests)
    target_compile_features(${PROJECT_NAME}_service_helper_lifecycle_tests
      PUBLIC cxx_std_17)
    target_compile_options(${PROJECT_NAME}_service_helper_lifecycle_tests PRIVATE
      -Wall -Wextra -Wpedantic -Werror
    )
    target_link_libraries(${PROJECT_NAME}_service_helper_lifecycle_tests
      ${catkin_LIBRARIES}
    )
    add_dependencies(${PROJECT_NAME}_service_helper_lifecycle_tests
      ${PROJECT_NAME}_service_helper)
  endif()

endif()
