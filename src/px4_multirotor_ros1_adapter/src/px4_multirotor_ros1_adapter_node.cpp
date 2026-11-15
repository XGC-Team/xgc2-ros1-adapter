#include "xgc_px4_multirotor_ros1_adapter/native_robot.hpp"
#include "xgc2_ros1_robot_adapter/ros1_binding.hpp"

int main(int argc, char **argv) {
  return xgc2_ros1_robot_adapter::RunRos1TypeServer(
      argc, argv, {{"xgc2-px4-multirotor-ros1-adapter", xgc_px4_multirotor_ros1_adapter::CreatePx4NativeRobot}});
}
