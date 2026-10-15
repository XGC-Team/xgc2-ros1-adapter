#include "xgc_scene/ros_outputs.hpp"

#include "xgc_scene/motion.hpp"
#include "xgc_scene/yaml_io.hpp"
#include <xgc2_math/scene/ellipse_ring.hpp>

#include <cmath>
#include <optional>
#include <unordered_map>

namespace xgc_scene {
namespace {
// JSON's numeric equality collapses signed zero. ROS messages retain those
// bits, so cache equality must also preserve the canonical scalar spelling.
bool sameProjectionValue(const Json& a, const Json& b) {
  if (a.type() != b.type() || a.size() != b.size()) return false;
  if (a.is_number_float()) {
    const auto x = a.get<double>(), y = b.get<double>();
    return x == y && (x != 0 || std::signbit(x) == std::signbit(y));
  }
  if (a.is_structured()) {
    auto left = a.begin(), right = b.begin();
    for (; left != a.end(); ++left, ++right) {
      if (a.is_object() && left.key() != right.key()) return false;
      if (!sameProjectionValue(left.value(), right.value())) return false;
    }
    return true;
  }
  return a == b;
}
void setPose(geometry_msgs::Pose& message, const Json& value) {
  message.position.x = value.at("position")[0].get<double>();
  message.position.y = value.at("position")[1].get<double>();
  message.position.z = value.at("position")[2].get<double>();
  message.orientation.x = value.at("orientation")[0].get<double>();
  message.orientation.y = value.at("orientation")[1].get<double>();
  message.orientation.z = value.at("orientation")[2].get<double>();
  message.orientation.w = value.at("orientation")[3].get<double>();
}
geometry_msgs::Point point(const Json& value) {
  geometry_msgs::Point result;
  result.x = value[0].get<double>(); result.y = value[1].get<double>(); result.z = value[2].get<double>();
  return result;
}
ros::Time time(double seconds) { ros::Time result; result.fromSec(seconds); return result; }
}  // namespace

xgc2_geometry_msgs::SceneObstacle obstacleMessage(const Json& item) {
  xgc2_geometry_msgs::SceneObstacle body;
  body.id = item.at("id").get<std::string>();
  body.name = item.at("name").get<std::string>();
  body.motion_type = item.at("motion").at("type").get<std::string>();
  body.dynamic = body.motion_type != "hold";
  setPose(body.pose, item.at("pose"));
  for (const auto& definition : item.at("parts")) {
    xgc2_geometry_msgs::ScenePart part;
    part.id = definition.at("id").get<std::string>();
    setPose(part.pose, definition.at("pose"));
    const auto& geometry = definition.at("geometry");
    part.geometry.type = geometry.at("type").get<std::string>();
    const auto size = geometry.value("size", Json::array({0, 0, 0}));
    part.geometry.size.x = size[0].get<double>();
    part.geometry.size.y = size[1].get<double>();
    part.geometry.size.z = size[2].get<double>();
    part.geometry.radius = geometry.value("radius", 0.0);
    part.geometry.height = geometry.value("height", 0.0);
    if (part.geometry.type == "ring") {
      part.geometry.radii[0] = geometry.at("radii")[0].get<double>();
      part.geometry.radii[1] = geometry.at("radii")[1].get<double>();
      part.geometry.tube_radius = geometry.at("tubeRadius").get<double>();
    }
    for (const auto& vertex : geometry.value("vertices", Json::array())) part.geometry.vertices.push_back(point(vertex));
    for (const auto& triangle : geometry.value("triangles", Json::array()))
      part.geometry.triangles.push_back(triangle.get<std::uint32_t>());
    part.color.r = definition.at("color")[0].get<float>();
    part.color.g = definition.at("color")[1].get<float>();
    part.color.b = definition.at("color")[2].get<float>();
    part.color.a = definition.at("color")[3].get<float>();
    body.parts.push_back(std::move(part));
  }
  return body;
}

xgc2_geometry_msgs::SceneObstacleState stateMessage(const Json& value) {
  xgc2_geometry_msgs::SceneObstacleState body;
  body.id = value.at("id").get<std::string>();
  setPose(body.pose, value.at("pose"));
  body.twist.linear.x = value.at("linear")[0].get<double>();
  body.twist.linear.y = value.at("linear")[1].get<double>();
  body.twist.linear.z = value.at("linear")[2].get<double>();
  body.twist.angular.x = value.at("angular")[0].get<double>();
  body.twist.angular.y = value.at("angular")[1].get<double>();
  body.twist.angular.z = value.at("angular")[2].get<double>();
  return body;
}

std::vector<visualization_msgs::Marker> partMarkers(
    const std::string& oid, const xgc2_geometry_msgs::ScenePart& part, const std::string& frame) {
  using Marker = visualization_msgs::Marker;
  const auto& geometry = part.geometry;
  Marker marker;
  marker.header.frame_id = frame;
  marker.ns = oid + "/" + part.id;
  marker.action = Marker::ADD;
  marker.pose = part.pose;
  marker.color = part.color;
  marker.frame_locked = true;
  marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
  if (geometry.type == "box") {
    marker.type = Marker::CUBE;
    marker.scale = geometry.size;
  } else if (geometry.type == "sphere") {
    marker.type = Marker::SPHERE;
    marker.scale.x = marker.scale.y = marker.scale.z = 2 * geometry.radius;
  } else if (geometry.type == "cylinder" || geometry.type == "capsule") {
    marker.type = Marker::CYLINDER;
    marker.scale.x = marker.scale.y = 2 * geometry.radius;
    marker.scale.z = geometry.height;
    if (geometry.type == "capsule") {
      std::vector<Marker> result;
      if (geometry.height > 0) result.push_back(marker);
      const auto& q = marker.pose.orientation;
      const Json quaternion = Json::array({q.x, q.y, q.z, q.w});
      const int count = geometry.height ? 2 : 1;
      for (int index = 0; index < count; ++index) {
        const int direction = count == 1 || index == 1 ? 1 : -1;
        const auto delta = rotate(quaternion, Json::array({0, 0, direction * geometry.height / 2}));
        Marker cap;
        cap.header = marker.header;
        cap.ns = marker.ns;
        cap.id = index + 1;
        cap.type = Marker::SPHERE;
        cap.action = marker.action;
        cap.pose = marker.pose;
        cap.pose.position.x += delta[0].get<double>();
        cap.pose.position.y += delta[1].get<double>();
        cap.pose.position.z += delta[2].get<double>();
        cap.scale.x = cap.scale.y = cap.scale.z = 2 * geometry.radius;
        cap.color = marker.color;
        cap.frame_locked = marker.frame_locked;
        result.push_back(std::move(cap));
      }
      return result;
    }
  } else if (geometry.type == "ring") {
    marker.type = Marker::POINTS;
    const double spacing = std::min(0.02, geometry.tube_radius * 0.5);
    marker.scale.x = marker.scale.y = spacing;
    const auto samples = xgc2_math::scene::sampleEllipticRingSurface(
        {geometry.radii[0], geometry.radii[1]}, geometry.tube_radius, spacing);
    for (const auto& sample : samples) {
      geometry_msgs::Point p;
      p.x = sample.x(); p.y = sample.y(); p.z = sample.z();
      marker.points.push_back(p);
    }
  } else if (geometry.type == "convex") {
    marker.type = Marker::TRIANGLE_LIST;
    for (const auto index : geometry.triangles) marker.points.push_back(geometry.vertices.at(index));
  }
  return {marker};
}

class SceneProjection::Impl {
 public:
  explicit Impl(const std::string& name_space) : prefix("xgc_scene_" + digest(name_space).substr(0, 12)) {}
  struct Body { Json value; xgc2_geometry_msgs::SceneObstacle message; };
  struct Markers { Json value; std::string frame; std::vector<visualization_msgs::Marker> messages; };
  struct Entry { Json value; std::optional<xgc2_geometry_msgs::SceneObstacleState> constant; };
  std::string prefix, epoch, parent;
  std::uint64_t revision = 0;
  Counters counts;
  std::unordered_map<std::string, std::shared_ptr<const Body>> bodies;
  std::unordered_map<std::string, std::shared_ptr<const Markers>> markers;
  std::unordered_map<std::string, std::string> frames;
  std::vector<Entry> entries;

  geometry_msgs::TransformStamped transform(const std::string& oid, const geometry_msgs::Pose& pose, const ros::Time& stamp) const {
    geometry_msgs::TransformStamped result;
    result.header.stamp = stamp;
    result.header.frame_id = parent;
    result.child_frame_id = frames.at(oid);
    result.transform.translation.x = pose.position.x;
    result.transform.translation.y = pose.position.y;
    result.transform.translation.z = pose.position.z;
    result.transform.rotation = pose.orientation;
    return result;
  }
};

SceneProjection::SceneProjection(const std::string& name_space) : impl_(new Impl(name_space)) {}
SceneProjection::~SceneProjection() = default;
const SceneProjection::Counters& SceneProjection::counters() const { return impl_->counts; }
const std::string& SceneProjection::frame(const std::string& oid) const { return impl_->frames.at(oid); }

xgc2_geometry_msgs::SceneSnapshot SceneProjection::snapshot(const Json& document, const std::string& epoch,
                                                          std::uint64_t revision, double stamp) {
  xgc2_geometry_msgs::SceneSnapshot result;
  result.header.stamp = time(stamp);
  result.header.frame_id = document.at("frame").get<std::string>();
  result.scene_id = document.at("id").get<std::string>();
  result.epoch = epoch;
  result.revision = revision;
  decltype(impl_->bodies) bodies;
  ++impl_->counts.snapshot_scans;
  for (const auto& item : document.at("obstacles")) {
    const auto id = item.at("id").get<std::string>();
    const auto cached = impl_->bodies.find(id);
    std::shared_ptr<const Impl::Body> body;
    if (cached != impl_->bodies.end() && sameProjectionValue(cached->second->value, item)) body = cached->second;
    else {
      ++impl_->counts.obstacle_messages;
      body = std::make_shared<const Impl::Body>(Impl::Body{item, obstacleMessage(item)});
    }
    result.obstacles.push_back(body->message);
    bodies[id] = std::move(body);
  }
  impl_->bodies = std::move(bodies);
  return result;
}

void SceneProjection::definition(const Json& document, const std::string& epoch, std::uint64_t revision,
                                 const xgc2_geometry_msgs::SceneSnapshot& snapshot, RosOutputs& outputs) {
  if (impl_->epoch != epoch || impl_->revision != revision) {
    impl_->epoch = epoch;
    impl_->revision = revision;
    impl_->parent = document.at("frame").get<std::string>();
    impl_->frames.clear();
    impl_->entries.clear();
    ++impl_->counts.frame_scans;
    for (const auto& item : document.at("obstacles")) {
      const auto id = item.at("id").get<std::string>();
      const bool held = item.at("motion").at("type") == "hold";
      impl_->frames[id] = impl_->prefix + "/" + epoch + (held ? "/fixed/" : "/moving/") + id;
      Impl::Entry entry{item, {}};
      if (held) {
        entry.constant = stateMessage(xgc_scene::state(item, 0.0, false));
        ++impl_->counts.hold_states;
      }
      impl_->entries.push_back(std::move(entry));
    }
  }
  outputs.snapshot(snapshot);
  std::vector<geometry_msgs::TransformStamped> fixed;
  visualization_msgs::MarkerArray markers;
  visualization_msgs::Marker remove;
  remove.action = visualization_msgs::Marker::DELETEALL;
  markers.markers.push_back(remove);
  decltype(impl_->markers) cache;
  ++impl_->counts.definition_scans;
  std::size_t index = 0;
  for (const auto& item : document.at("obstacles")) {
    const auto& body = snapshot.obstacles.at(index++);
    if (!body.dynamic) fixed.push_back(impl_->transform(body.id, body.pose, snapshot.header.stamp));
    const auto& frame = impl_->frames.at(body.id);
    const auto found = impl_->markers.find(body.id);
    std::shared_ptr<const Impl::Markers> cached;
    if (found != impl_->markers.end() && sameProjectionValue(found->second->value, item) && found->second->frame == frame)
      cached = found->second;
    else {
      Impl::Markers built{item, frame, {}};
      for (const auto& part : body.parts) {
        ++impl_->counts.part_markers;
        const auto messages = partMarkers(body.id, part, frame);
        built.messages.insert(built.messages.end(), messages.begin(), messages.end());
      }
      cached = std::make_shared<const Impl::Markers>(std::move(built));
    }
    markers.markers.insert(markers.markers.end(), cached->messages.begin(), cached->messages.end());
    cache[body.id] = std::move(cached);
  }
  impl_->markers = std::move(cache);
  if (!fixed.empty()) outputs.staticTransforms(fixed);
  outputs.markers(markers);
}

void SceneProjection::tick(double stamp, double elapsed, bool playing, RosOutputs& outputs) {
  xgc2_geometry_msgs::SceneState message;
  message.header.stamp = time(stamp);
  message.header.frame_id = impl_->parent;
  message.epoch = impl_->epoch;
  message.revision = impl_->revision;
  message.playing = playing;
  message.scene_time = elapsed;
  std::vector<geometry_msgs::TransformStamped> transforms;
  ++impl_->counts.state_scans;
  for (const auto& entry : impl_->entries) {
    if (entry.constant) message.obstacles.push_back(*entry.constant);
    else {
      ++impl_->counts.moving_states;
      auto body = stateMessage(xgc_scene::state(entry.value, elapsed, playing));
      transforms.push_back(impl_->transform(body.id, body.pose, message.header.stamp));
      message.obstacles.push_back(std::move(body));
    }
  }
  if (!transforms.empty()) outputs.dynamicTransforms(transforms);
  outputs.state(message);
}

}  // namespace xgc_scene
