// Pair model IMU with orientation from exactly the same integration step.
#pragma once
#include <cmath>
#include <xgc-robotics-interfaces/robotics_interfaces_v1.h>

inline bool matched_sim_imu(const xgc_pose_v1& pose, const xgc_imu_v1& imu,
                            double last_stamp) {
  if (!std::isfinite(imu.stamp) || imu.stamp <= last_stamp ||
      imu.stamp != pose.stamp) return false;
  double norm = 0.0;
  for (double q : pose.q_wxyz) {
    if (!std::isfinite(q)) return false;
    norm += q * q;
  }
  if (std::abs(norm - 1.0) > 1e-6) return false;
  for (int i = 0; i != 3; ++i)
    if (!std::isfinite(imu.accel[i]) || !std::isfinite(imu.gyro[i])) return false;
  return true;
}
