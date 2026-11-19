#include <csignal>
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

rosbag::RecorderOptions record_options(int argc, char** argv) {
  rosbag::RecorderOptions options;
  for (int i = 1; i < argc; ++i) {
    const std::string_view arg(argv[i]);
    if (arg == "--split") {
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

void on_terminate(int) { ros::requestShutdown(); }

}  // namespace

int main(int argc, char** argv) {
  try {
    ros::init(argc, argv, "record", ros::init_options::NoSimTime);
    const auto options = record_options(argc, argv);
    std::signal(SIGTERM, on_terminate);
    ClockReceipt clock_receipt;
    ros::NodeHandle node;
    ros::Subscriber clock;
    bool simulated = false;
    node.param("/use_sim_time", simulated, false);
    if (simulated) {
      ros::Time::setNow(ros::Time());
      ros::SubscribeOptions options;
      options.init<rosgraph_msgs::Clock>("/clock", 1,
          [](const rosgraph_msgs::Clock::ConstPtr& tick) { ros::Time::setNow(tick->clock); });
      options.callback_queue = &clock_receipt;
      clock = node.subscribe(options);
    }
    rosbag::Recorder recorder(options);
    return recorder.run();
  } catch (const std::exception& failure) {
    std::cerr << "rosbag simulation recorder: " << failure.what() << '\n';
    return 1;
  }
}
