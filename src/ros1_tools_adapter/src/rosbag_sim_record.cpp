#include "xsim_receipt_clock.hpp"

#include <atomic>
#include <csignal>
#include <dlfcn.h>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

#include <ros/callback_queue_interface.h>
#include <ros/ros.h>
#include <rosbag/recorder.h>
#include <rosgraph_msgs/Clock.h>

namespace {

xgc::recording::XSimReceiptClock receipt_clock;
std::atomic<bool> clock_retired{false};
volatile sig_atomic_t external_stop = 0;

// One existing subscription applies the authoritative simulation clock when
// roscpp receives it. The separate internal spinner can otherwise fall
// behind the recorder callbacks on the same /clock connection.
class ClockReceipt final : public ros::CallbackQueueInterface {
 public:
  void addCallback(const ros::CallbackInterfacePtr& callback, uint64_t) override {
    callback->call();
  }

  void removeByID(uint64_t) override {}
};

uint64_t units(std::string_view text, uint64_t scale) {
  size_t end = 0;
  const auto value = std::stoull(std::string(text), &end);
  if (end != text.size() || value > UINT64_MAX / scale)
    throw std::invalid_argument("invalid recorder size");
  return value * scale;
}

rosbag::RecorderOptions record_options(int argc, char** argv, std::string &world_clock) {
  rosbag::RecorderOptions options;
  for (int i = 1; i < argc; ++i) {
    const std::string_view arg(argv[i]);
    if (arg == "--world-receipt-clock" && i + 1 < argc && world_clock.empty()) {
      world_clock = argv[++i];
    } else if (arg == "--split") {
      options.split = true;
    } else if (arg.substr(0, 7) == "--size=") {
      options.max_size = units(arg.substr(7), 1024 * 1024);
    } else if (arg.substr(0, 12) == "--min-space=" && arg.back() == 'G') {
      options.min_space_str = arg.substr(12);
      options.min_space = units(arg.substr(12, arg.size() - 13), 1024ULL * 1024 * 1024);
    } else if (arg == "--lz4") {
      options.compression = rosbag::compression::LZ4;
    } else if (arg == "--bz2") {
      options.compression = rosbag::compression::BZ2;
    } else if (arg == "--repeat-latched") {
      options.repeat_latched = true;
    } else if ((arg == "--output-name" || arg == "--output-prefix") && i + 1 < argc) {
      options.append_date = arg == "--output-prefix";
      options.prefix = argv[++i];
    } else if (arg == "--all") {
      options.record_all = true;
    } else if (arg == "--exclude" && i + 1 < argc) {
      options.exclude_regex = boost::regex(argv[++i]);
      options.do_exclude = true;
    } else if (!arg.empty() && arg.front() == '/') {
      options.topics.emplace_back(arg);
    } else {
      throw std::invalid_argument("unsupported native recorder argument: " + std::string(arg));
    }
  }
  return options;
}

void on_terminate(int) {
  external_stop = 1;
  ros::requestShutdown();
}

}  // namespace

// The isolated pure-simulation recorder owns this ROS time provider. The
// Noetic recorder still stamps where its original code calls Time::now().
// No message fields, TCP delivery or callback ordering enter this clock.
ros::Time ros::Time::now() {
  if (const auto state = receipt_clock.stamp()) {
    if ((state & xgc::recording::XSimReceiptClock::retired) && !clock_retired.exchange(true))
      ros::requestShutdown();
    ros::Time current;
    current.fromNSec(state & ~xgc::recording::XSimReceiptClock::retired);
    return current;
  }
  static auto stock_now = reinterpret_cast<ros::Time (*)()>(dlsym(RTLD_NEXT, "_ZN3ros4Time3nowEv"));
  if (!stock_now) throw std::runtime_error("native Noetic time provider is unavailable");
  return stock_now();
}

int main(int argc, char** argv) {
  try {
    ros::init(argc, argv, "record", ros::init_options::NoSimTime | ros::init_options::NoSigintHandler);
    std::string world_clock;
    const auto options = record_options(argc, argv, world_clock);
    std::signal(SIGINT, on_terminate);
    std::signal(SIGTERM, on_terminate);
    ClockReceipt clock_receipt;
    ros::NodeHandle node;
    ros::Subscriber clock;
    bool simulated = false;
    node.param("/use_sim_time", simulated, false);
    if (!world_clock.empty()) {
      if (!simulated) throw std::runtime_error("receipt clock requires simulation time");
      receipt_clock.open(world_clock);
      ros::Time initial;
      initial.fromNSec(receipt_clock.stamp() & ~xgc::recording::XSimReceiptClock::retired);
      ros::Time::setNow(initial);  // Let the native ROS validity wait finish.
    } else if (simulated) {
      ros::Time::setNow(ros::Time());
      ros::SubscribeOptions options;
      options.init<rosgraph_msgs::Clock>("/clock", 1,
          [](const rosgraph_msgs::Clock::ConstPtr& tick) { ros::Time::setNow(tick->clock); });
      options.callback_queue = &clock_receipt;
      clock = node.subscribe(options);
    }
    rosbag::Recorder recorder(options);
    const auto result = recorder.run();
    if (clock_retired && !external_stop) {
      std::cerr << "simulation world retired while recorder was running" << '\n';
      return 1;
    }
    return result;
  } catch (const std::exception& failure) {
    std::cerr << "rosbag simulation recorder: " << failure.what() << '\n';
    return 1;
  }
}
