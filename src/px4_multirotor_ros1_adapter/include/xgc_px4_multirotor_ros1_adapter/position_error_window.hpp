#pragma once

#include <algorithm>
#include <cmath>
#include <deque>
#include <optional>
#include <geometry_msgs/PoseStamped.h>

namespace xgc_px4_multirotor_ros1_adapter {

// Both inputs must use the same ROS clock and world frame. The delay describes
// measurement time -> mocap header time; it never changes the forwarded pose.
class PositionErrorWindow {
public:
  struct Result {
    ros::Time stamp;
    std::string frame;
    double meters;
  };

  explicit PositionErrorWindow(double delay_seconds = 0.0)
      : delay_(delay_seconds) {}

  void clear() {
    local_.clear();
    pending_.clear();
    last_mocap_stamp_ = ros::Time();
    latest_.reset();
  }

  bool local(const geometry_msgs::PoseStamped &pose) {
    if (!valid(pose)) {
      clear();
      return false;
    }
    if (!local_.empty() &&
        (pose.header.stamp < local_.back().stamp ||
         pose.header.frame_id != local_.back().frame))
      clear();
    const Sample sample{pose.header.stamp, pose.header.frame_id,
                        pose.pose.position};
    if (!local_.empty() && pose.header.stamp == local_.back().stamp)
      local_.back() = sample;
    else
      local_.push_back(sample);
    while (local_.size() > 512 ||
           (local_.size() > 1 &&
            (local_.back().stamp - local_.front().stamp).toSec() > 2.0))
      local_.pop_front();
    return match();
  }

  bool mocap(const geometry_msgs::PoseStamped &pose) {
    if (!valid(pose) || pose.header.stamp.toSec() < delay_.toSec()) {
      clear();
      return false;
    }
    if (!last_mocap_stamp_.isZero() && pose.header.stamp < last_mocap_stamp_)
      clear();
    if (pose.header.stamp == last_mocap_stamp_)
      return false;
    last_mocap_stamp_ = pose.header.stamp;
    latest_.reset();
    pending_.push_back({pose.header.stamp - delay_, pose.header.frame_id,
                        pose.pose.position});
    while (pending_.size() > 256 ||
           (pending_.size() > 1 &&
            (pending_.back().stamp - pending_.front().stamp).toSec() > 2.0))
      pending_.pop_front();
    return match();
  }

  const std::optional<Result> &latest() const { return latest_; }

private:
  struct Sample {
    ros::Time stamp;
    std::string frame;
    geometry_msgs::Point position;
  };
  static bool valid(const geometry_msgs::PoseStamped &pose) {
    return !pose.header.stamp.isZero() && !pose.header.frame_id.empty() &&
           std::isfinite(pose.pose.position.x) &&
           std::isfinite(pose.pose.position.y) &&
           std::isfinite(pose.pose.position.z);
  }
  bool match() {
    bool updated = false;
    while (!pending_.empty() && !local_.empty() &&
           pending_.front().stamp <= local_.back().stamp) {
      const auto sample = pending_.front();
      pending_.pop_front();
      const auto right = std::lower_bound(
          local_.begin(), local_.end(), sample.stamp,
          [](const Sample &value, const ros::Time &stamp) {
            return value.stamp < stamp;
          });
      if (right == local_.end() || right->frame != sample.frame ||
          (right == local_.begin() && right->stamp != sample.stamp))
        continue;
      auto position = right->position;
      if (right->stamp != sample.stamp) {
        const auto &left = *std::prev(right);
        const double gap = (right->stamp - left.stamp).toSec();
        // 100 ms accommodates the configured 15 Hz local-position stream,
        // but does not interpolate across an outage or a reset.
        if (left.frame != sample.frame || gap <= 0.0 || gap > 0.1)
          continue;
        const double alpha = (sample.stamp - left.stamp).toSec() / gap;
        position.x = left.position.x + alpha * (position.x - left.position.x);
        position.y = left.position.y + alpha * (position.y - left.position.y);
        position.z = left.position.z + alpha * (position.z - left.position.z);
      }
      const double distance = std::hypot(
          std::hypot(position.x - sample.position.x,
                     position.y - sample.position.y),
          position.z - sample.position.z);
      if (std::isfinite(distance)) {
        latest_ = Result{sample.stamp, sample.frame, distance};
        updated = true;
      }
    }
    return updated;
  }

  ros::Duration delay_;
  std::deque<Sample> local_, pending_;
  ros::Time last_mocap_stamp_;
  std::optional<Result> latest_;
};

} // namespace xgc_px4_multirotor_ros1_adapter
