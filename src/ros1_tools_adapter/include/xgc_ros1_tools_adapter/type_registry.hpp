#pragma once

#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros_babel_fish/babel_fish.h>
#include <ros_babel_fish/message_description.h>

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

namespace xgc_ros1_tools_adapter {

class TypeRegistry {
 public:
  TypeRegistry();

  ros_babel_fish::Message::Ptr createMessage(const std::string& type);
  ros_babel_fish::ServiceDescription::ConstPtr resolveService(
      const std::string& type);
  ros_babel_fish::Message::Ptr createServiceRequest(
      const ros_babel_fish::ServiceDescription::ConstPtr& description) const;
  ros_babel_fish::Message::Ptr createServiceResponse(
      const ros_babel_fish::ServiceDescription::ConstPtr& description) const;
  ros_babel_fish::BabelFishMessage::Ptr serialize(
      const ros_babel_fish::Message::ConstPtr& message);
  ros::Publisher advertise(ros::NodeHandle& node_handle,
                           const std::string& type, const std::string& topic,
                           std::uint32_t queue_size, bool latch);

 private:
  std::shared_ptr<ros_babel_fish::DescriptionProvider> provider_;
  ros_babel_fish::BabelFish babel_fish_;
  std::mutex resolver_mutex_;
};

}  // namespace xgc_ros1_tools_adapter
