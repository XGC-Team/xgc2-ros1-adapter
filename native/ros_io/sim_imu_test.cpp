#include "sim_imu.hpp"
#include <cassert>
#include <limits>

int main() {
  xgc_pose_v1 pose{};
  pose.stamp = 1.25;
  // 90 degrees about body X: retain the full non-yaw attitude.
  pose.q_wxyz[0] = pose.q_wxyz[1] = std::sqrt(0.5);
  xgc_imu_v1 imu{};
  imu.stamp = pose.stamp;
  imu.accel[1] = 9.8066;
  imu.gyro[0] = 0.3;
  assert(matched_sim_imu(pose, imu, 1.0));
  assert(!matched_sim_imu(pose, imu, imu.stamp));
  assert(!matched_sim_imu(pose, imu, 2.0));
  pose.stamp += 0.01;
  assert(!matched_sim_imu(pose, imu, 1.0));
  pose.stamp = imu.stamp;
  pose.q_wxyz[1] = 0;
  assert(!matched_sim_imu(pose, imu, 1.0));
  pose.q_wxyz[1] = std::sqrt(0.5);
  imu.accel[2] = std::numeric_limits<double>::quiet_NaN();
  assert(!matched_sim_imu(pose, imu, 1.0));
  imu.accel[2] = 0;
  imu.gyro[1] = std::numeric_limits<double>::infinity();
  assert(!matched_sim_imu(pose, imu, 1.0));
}
