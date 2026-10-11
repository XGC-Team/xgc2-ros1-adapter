find_package(XgcXrpc 0.2.0 REQUIRED CONFIG COMPONENTS http)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIDAR_CONTROL_JSONCPP REQUIRED IMPORTED_TARGET jsoncpp)
target_link_libraries(xgc_scene_lidar PUBLIC XgcXrpc::http PRIVATE PkgConfig::LIDAR_CONTROL_JSONCPP)
