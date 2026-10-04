// roscpp's TopicManager::unadvertise finds a Publication under advertised_topics_mutex_,
// releases the lock, and later erases it with the iterator from the first
// critical section (ros_comm clients/roscpp/src/libros/topic_manager.cpp,
// `advertised_topics_.erase(i)`; identical in 1.16.0 and 1.17.x). When another
// thread advertises or unadvertises in between, the iterator is stale: the
// erase removes another topic's Publication, and the next publish on that
// topic dereferences a null Publication (TopicManager::publish does not check
// the lookup) -- a segmentation fault in Publisher::publish.
//
// Modes (a ROS master must be reachable, like the other live checks):
//   raw      concurrent Publisher::shutdown from many threads, as the per-robot
//            edges did at Stop. Informational: it demonstrates the defect and is
//            expected to fault with roscpp as released.
//   guarded  the same teardown through xgc_ros_edge::publisher_lifecycle_mutex(),
//            as RosIo::activate/shutdown do. Must never fault.
// run-publisher-lifecycle-test.sh runs each mode in fresh processes.
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Empty.h>

#include "ros_edge.hpp"

int main(int argc, char** argv) {
  const bool guarded = argc > 1 && std::strcmp(argv[1], "guarded") == 0;
  const int threads = argc > 2 ? std::atoi(argv[2]) : 12;
  const int topics = argc > 3 ? std::atoi(argv[3]) : 12;
  ros::init(argc, argv, guarded ? "publisher_lifecycle_guarded" : "publisher_lifecycle_raw",
            ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  ros::NodeHandle nh;
  std::vector<std::vector<ros::Publisher>> owned(threads);
  std::atomic<int> advertised{0};
  std::atomic<bool> go{false}, torn_down{false};
  std::vector<std::thread> workers;
  for (int t = 0; t != threads; ++t) {
    workers.emplace_back([&, t] {
      {
        std::unique_lock<std::mutex> lock(xgc_ros_edge::publisher_lifecycle_mutex(), std::defer_lock);
        if (guarded) lock.lock();
        for (int k = 0; k != topics; ++k)
          owned[t].push_back(nh.advertise<std_msgs::Empty>("/lifecycle_" + std::to_string(t) + "/" + std::to_string(k), 1));
      }
      ++advertised;
      while (!go) std::this_thread::yield();
      for (auto& publisher : owned[t]) {
        std::unique_lock<std::mutex> lock(xgc_ros_edge::publisher_lifecycle_mutex(), std::defer_lock);
        if (guarded) lock.lock();
        publisher.shutdown();
      }
    });
  }
  while (advertised != threads) std::this_thread::yield();
  // Live topics nobody unadvertises, advertised last: a stale erase moves
  // later elements into the erased position, so these are the ones it removes.
  std::vector<ros::Publisher> probes;
  for (int p = 0; p != threads * 2; ++p) probes.push_back(nh.advertise<std_msgs::Empty>("/lifecycle_probe/" + std::to_string(p), 1));
  std::thread publisher_thread([&] {
    const std_msgs::Empty message;
    while (!torn_down) for (auto& probe : probes) probe.publish(message);
  });
  go = true;
  for (auto& worker : workers) worker.join();
  torn_down = true;
  publisher_thread.join();
  const std_msgs::Empty message;
  for (int i = 0; i != 100; ++i) for (auto& probe : probes) probe.publish(message);
  std::puts("survived");
  return 0;
}
