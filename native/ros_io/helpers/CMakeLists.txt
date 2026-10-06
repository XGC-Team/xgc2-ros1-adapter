cmake_minimum_required(VERSION 3.16)
project(XgcRosIoHelpers VERSION 0.1.0 LANGUAGES CXX)

find_package(XgcRoboticsInterfaces CONFIG REQUIRED)
find_package(XgcRuntimeSDK CONFIG REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp)
include(CMakePackageConfigHelpers)
set(CMAKE_INSTALL_INCLUDEDIR "include" CACHE PATH "Header installation directory")
set(CMAKE_INSTALL_DATADIR "share" CACHE PATH "Architecture-independent data directory")
# catkin/GNUInstallDirs may pre-create an empty cache default.
if(NOT CMAKE_INSTALL_DATADIR)
  set(CMAKE_INSTALL_DATADIR "share")
endif()
set(helper_package_dir "${CMAKE_INSTALL_DATADIR}/cmake/XgcRosIoHelpers")

add_library(xgc_ros_io_helpers INTERFACE)
add_library(XgcRosIo::Helpers ALIAS xgc_ros_io_helpers)
set_target_properties(xgc_ros_io_helpers PROPERTIES EXPORT_NAME Helpers)
target_include_directories(xgc_ros_io_helpers INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/xgc-ros-io>)
target_link_libraries(xgc_ros_io_helpers INTERFACE XgcRoboticsInterfaces::Interfaces)

# One implementation of the existing process ROS init/gate state; no plugin,
# threads, registry, or algorithm library is defined by this utility.
add_library(xgc_ros_edge SHARED ../ros_edge.cpp)
add_library(XgcRosIo::Edge ALIAS xgc_ros_edge)
set_target_properties(xgc_ros_edge PROPERTIES EXPORT_NAME Edge)
target_compile_features(xgc_ros_edge PUBLIC cxx_std_17)
target_include_directories(xgc_ros_edge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/xgc-ros-io>
  ${catkin_INCLUDE_DIRS})
target_link_libraries(xgc_ros_edge PUBLIC XgcRuntime::SDK ${catkin_LIBRARIES})
install(TARGETS xgc_ros_edge EXPORT XgcRosIoEdgeTargets
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)
install(EXPORT XgcRosIoEdgeTargets
  NAMESPACE XgcRosIo:: DESTINATION "${helper_package_dir}")
install(FILES ../ros_edge.hpp DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/xgc-ros-io")

install(FILES ../ros_slice.hpp
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/xgc-ros-io")
install(TARGETS xgc_ros_io_helpers EXPORT XgcRosIoHelpersTargets)
install(EXPORT XgcRosIoHelpersTargets
  NAMESPACE XgcRosIo:: DESTINATION "${helper_package_dir}")
configure_package_config_file(cmake/XgcRosIoHelpersConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/XgcRosIoHelpersConfig.cmake"
  INSTALL_DESTINATION "${helper_package_dir}")
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/XgcRosIoHelpersConfigVersion.cmake"
  VERSION "${PROJECT_VERSION}" COMPATIBILITY SameMajorVersion ARCH_INDEPENDENT)
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/XgcRosIoHelpersConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/XgcRosIoHelpersConfigVersion.cmake"
  DESTINATION "${helper_package_dir}")
