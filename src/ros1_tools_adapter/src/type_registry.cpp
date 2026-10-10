#include "xgc_ros1_tools_adapter/type_registry.hpp"

#include <ros_babel_fish/exceptions/babel_fish_exception.h>
#include <ros_babel_fish/generation/message_creation.h>
#include <ros_babel_fish/generation/providers/integrated_description_provider.h>

#include <exception>
#include <utility>

#include "xgc_ros1_tools_adapter/error.hpp"

namespace xgc_ros1_tools_adapter {

namespace {

std::string typeFailure(const std::string& kind, const std::string& type,
                        const std::exception& exception) {
  return "unable to resolve ROS1 " + kind + " type '" + type +
         "': " + exception.what();
}

}  // namespace

TypeRegistry::TypeRegistry()
    : provider_(
          std::make_shared<ros_babel_fish::IntegratedDescriptionProvider>()),
      babel_fish_(provider_) {}

ros_babel_fish::Message::Ptr TypeRegistry::createMessage(
    const std::string& type) {
  std::lock_guard<std::mutex> lock(resolver_mutex_);
  try {
    return babel_fish_.createMessage(type);
  } catch (const std::exception& exception) {
    permanentError("message_type_not_found",
                   typeFailure("message", type, exception));
  }
}

ros_babel_fish::ServiceDescription::ConstPtr TypeRegistry::resolveService(
    const std::string& type) {
  std::lock_guard<std::mutex> lock(resolver_mutex_);
  try {
    auto description = provider_->getServiceDescription(type);
    if (!description) {
      permanentError("service_type_not_found",
                     "unable to resolve ROS1 service type '" + type + "'");
    }
    return description;
  } catch (const Ros1ToolsError&) {
    throw;
  } catch (const std::exception& exception) {
    permanentError("service_type_not_found",
                   typeFailure("service", type, exception));
  }
}

ros_babel_fish::Message::Ptr TypeRegistry::createServiceRequest(
    const ros_babel_fish::ServiceDescription::ConstPtr& description) const {
  if (!description || !description->request ||
      !description->request->message_template) {
    permanentError("invalid_service_description",
                   "ROS1 service request description is incomplete");
  }
  return ros_babel_fish::createEmptyMessageFromTemplate(
      description->request->message_template);
}

ros_babel_fish::Message::Ptr TypeRegistry::createServiceResponse(
    const ros_babel_fish::ServiceDescription::ConstPtr& description) const {
  if (!description || !description->response ||
      !description->response->message_template) {
    permanentError("invalid_service_description",
                   "ROS1 service response description is incomplete");
  }
  return ros_babel_fish::createEmptyMessageFromTemplate(
      description->response->message_template);
}

ros_babel_fish::BabelFishMessage::Ptr TypeRegistry::serialize(
    const ros_babel_fish::Message::ConstPtr& message) {
  if (!message) {
    permanentError("invalid_message", "ROS1 message is empty");
  }
  std::lock_guard<std::mutex> lock(resolver_mutex_);
  try {
    return babel_fish_.translateMessage(message);
  } catch (const std::exception& exception) {
    permanentError(
        "message_serialization_failed",
        std::string("unable to serialize ROS1 message: ") + exception.what());
  }
}

ros::Publisher TypeRegistry::advertise(ros::NodeHandle& node_handle,
                                       const std::string& type,
                                       const std::string& topic,
                                       std::uint32_t queue_size, bool latch) {
  std::lock_guard<std::mutex> lock(resolver_mutex_);
  try {
    return babel_fish_.advertise(node_handle, type, topic, queue_size, latch);
  } catch (const std::exception& exception) {
    permanentError("publisher_advertise_failed",
                   "unable to advertise ROS1 topic '" + topic + "' as '" +
                       type + "': " + exception.what());
  }
}

}  // namespace xgc_ros1_tools_adapter
