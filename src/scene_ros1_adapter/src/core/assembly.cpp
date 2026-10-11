#include "xgc_scene/assembly.hpp"

#include "placement_json.hpp"
#include "scalars.hpp"
#include "xgc_scene/document.hpp"
#include "xgc_scene/json_io.hpp"
#include "xgc_scene/yaml_io.hpp"

#include <algorithm>
#include <set>
#include <utility>

namespace xgc_scene {
namespace {

std::string typeName(const Json& value) {
  if (value.is_null()) return "NoneType";
  if (value.is_boolean()) return "bool";
  if (value.is_number_integer()) return "int";
  if (value.is_number_float()) return "float";
  if (value.is_string()) return "str";
  return value.is_array() ? "list" : "dict";
}

const Json& item(const Json& value, const std::string& key) {
  if (!value.is_object()) {
    if (value.is_array()) throw SceneError("list indices must be integers or slices, not str");
    if (value.is_string()) throw SceneError("string indices must be integers");
    throw SceneError("'" + typeName(value) + "' object is not subscriptable");
  }
  const auto found = value.find(key);
  if (found == value.end()) throw SceneError(detail::pythonRepr(Json(key)));
  return *found;
}

Json get(const Json& value, const std::string& key, const Json& fallback = nullptr) {
  if (!value.is_object()) throw SceneError("'" + typeName(value) + "' object has no attribute 'get'");
  return value.value(key, fallback);
}

std::string strip(const Json& value) {
  if (!value.is_string()) throw SceneError("'" + typeName(value) + "' object has no attribute 'strip'");
  const auto characters = detail::utf8Characters(value.get_ref<const std::string&>());
  auto first = characters.begin(), last = characters.end();
  const auto whitespace = [](const std::string& c) { return detail::pythonWhitespace(detail::codepoint(c)); };
  while (first != last && whitespace(*first)) ++first;
  while (first != last && whitespace(*std::prev(last))) --last;
  std::string result;
  for (; first != last; ++first) result += *first;
  return result;
}

Json yaml(const Json& source) {
  if (!source.is_string()) throw SceneError("'" + typeName(source) + "' object has no attribute 'read'");
  return parseYaml(source.get_ref<const std::string&>());
}

// The CLI needs json.loads' diagnostics and parse_constant behavior, whereas
// the core's general parseJson deliberately accepts nonfinite numeric literals.
// Decode strings and numbers with that existing primitive; keep the JSON
// grammar and object-pairs hook here so errors retain their Python positions.
class AssemblyInput {
 public:
  explicit AssemblyInput(const std::string& bytes) : bytes_(bytes) {}

  Json decode() {
    if (bytes_.compare(0, 3, "\xef\xbb\xbf") == 0) at_ = begin_ = 3;
    Json result = value();
    whitespace();
    if (at_ != bytes_.size()) error("Extra data", at_);
    return result;
  }

 private:
  const std::string& bytes_;
  std::size_t at_ = 0;
  std::size_t begin_ = 0;

  [[noreturn]] void error(const std::string& message, std::size_t at) const {
    std::size_t line = 1, column = 1, characters = 0;
    for (auto i = begin_; i < at; ++i) {
      if ((static_cast<unsigned char>(bytes_[i]) & 0xc0) == 0x80) continue;
      ++characters;
      if (bytes_[i] == '\n') { ++line; column = 1; }
      else ++column;
    }
    throw SceneError(message + ": line " + std::to_string(line) + " column " + std::to_string(column) +
                     " (char " + std::to_string(characters) + ")");
  }
  char current() const { return at_ < bytes_.size() ? bytes_[at_] : '\0'; }
  void whitespace() {
    while (at_ < bytes_.size() && std::string(" \t\r\n").find(bytes_[at_]) != std::string::npos) ++at_;
  }
  bool take(char c) {
    if (at_ < bytes_.size() && bytes_[at_] == c) { ++at_; return true; }
    return false;
  }
  bool digit(std::size_t at) const { return at < bytes_.size() && bytes_[at] >= '0' && bytes_[at] <= '9'; }

  Json string() {
    const auto start = at_++;
    while (at_ < bytes_.size()) {
      const auto c = static_cast<unsigned char>(bytes_[at_++]);
      if (c == '"') return parseJson(bytes_.substr(start, at_ - start));
      if (c < 0x20) error("Invalid control character at", at_ - 1);
      if (c != '\\') continue;
      const auto escape = at_ - 1;
      if (at_ == bytes_.size()) break;
      if (bytes_[at_] == 'u') {
        ++at_;
        for (int i = 0; i < 4; ++i) {
          if (at_ == bytes_.size() || std::string("0123456789abcdefABCDEF").find(bytes_[at_]) == std::string::npos)
            error("Invalid \\uXXXX escape", escape + 1);
          ++at_;
        }
      } else {
        if (std::string("\"\\/bfnrt").find(bytes_[at_]) == std::string::npos) error("Invalid \\escape", escape);
        ++at_;
      }
    }
    error("Unterminated string starting at", start);
  }

  Json value() {
    whitespace();
    if (current() == '"') return string();
    if (take('{')) {
      std::vector<std::pair<Json, Json>> pairs;
      whitespace();
      if (take('}')) return uniqueObject(pairs);
      while (true) {
        if (current() != '"') error("Expecting property name enclosed in double quotes", at_);
        Json key = string();
        whitespace();
        if (!take(':')) error("Expecting ':' delimiter", at_);
        Json field = value();
        pairs.emplace_back(std::move(key), std::move(field));
        whitespace();
        if (take('}')) return uniqueObject(pairs);
        if (!take(',')) error("Expecting ',' delimiter", at_);
        whitespace();
      }
    }
    if (take('[')) {
      Json result = Json::array();
      whitespace();
      if (take(']')) return result;
      while (true) {
        result.push_back(value());
        whitespace();
        if (take(']')) return result;
        if (!take(',')) error("Expecting ',' delimiter", at_);
      }
    }
    for (const auto* constant : {"NaN", "Infinity", "-Infinity"}) {
      if (bytes_.compare(at_, std::char_traits<char>::length(constant), constant) == 0)
        throw SceneError("Nonfinite JSON");
    }
    for (const auto* literal : {"null", "true", "false"}) {
      const auto length = std::char_traits<char>::length(literal);
      if (bytes_.compare(at_, length, literal) == 0) {
        at_ += length;
        return parseJson(literal);
      }
    }
    const auto start = at_;
    take('-');
    if (take('0')) {}
    else if (current() >= '1' && current() <= '9') { while (digit(at_)) ++at_; }
    else error("Expecting value", start);
    if (current() == '.' && digit(at_ + 1)) { at_ += 2; while (digit(at_)) ++at_; }
    if (current() == 'e' || current() == 'E') {
      auto exponent = at_ + 1;
      if (exponent < bytes_.size() && (bytes_[exponent] == '+' || bytes_[exponent] == '-')) ++exponent;
      if (digit(exponent)) { at_ = exponent + 1; while (digit(at_)) ++at_; }
    }
    return parseJson(bytes_.substr(start, at_ - start));
  }
};

}  // namespace

Json assemble(const Json& request, const Json& documents, const Json& extrinsic) {
  const auto fidelity = get(request, "fidelity");
  if (fidelity != "geometry-only" && fidelity != "gazebo-world")
    throw SceneError("fidelity must be geometry-only or gazebo-world");
  const auto task = identifier(strip(get(request, "task", "")), "Task");
  if (task.size() > 64) throw SceneError("Task must have at most 64 characters");
  const auto elements = get(request, "elements");
  if (!elements.is_array() || elements.size() < 2 || elements.size() > 64)
    throw SceneError("An assembled world needs 2 to 64 elements");
  if (!documents.is_structured() && !documents.is_string())
    throw SceneError("object of type '" + typeName(documents) + "' has no len()");
  Json characters;
  if (documents.is_string()) characters = detail::utf8Characters(documents.get_ref<const std::string&>());
  const auto& sources = documents.is_string() ? characters : documents;
  if (sources.size() != elements.size()) throw SceneError("An assembled world needs 2 to 64 elements");

  Json obstacles = Json::array();
  std::set<std::string> ids;
  std::string frame;
  auto source = sources.begin();
  for (const auto& element : elements) {
    const auto eid = identifier(item(element, "id"), "Element ID");
    if (eid.find('.') != std::string::npos || !ids.insert(eid).second)
      throw SceneError("Element IDs must be unique and contain no dots");
    const xgc2_math::scene::ElementPlacement placement{
        number(get(element, "x", 0), "x", -1e4, 1e4), number(get(element, "y", 0), "y", -1e4, 1e4),
        number(get(element, "yaw", 0), "yaw", -1e4, 1e4)};
    const auto scene = document(yaml(sources.is_object() ? Json(source.key()) : *source));
    ++source;
    if (!frame.empty() && frame != scene.at("frame")) throw SceneError("All elements must use the same world frame");
    frame = scene.at("frame").get<std::string>();
    for (const auto& original : scene.at("obstacles")) {
      Json placed = original;
      placed["id"] = identifier(eid + "." + original.at("id").get<std::string>());
      placed["name"] = eid + " " + original.at("name").get<std::string>();
      placed["pose"] = detail::fromPose(xgc2_math::scene::composePlacement(placement.pose(), detail::toPose(original.at("pose"))));
      auto& movement = placed["motion"];
      const auto motion = xgc2_math::scene::placeMotion(placement, detail::toMotion(movement));
      switch (motion.type) {
        case xgc2_math::scene::Motion::kHold: break;
        case xgc2_math::scene::Motion::kConstantTwist:
          movement["linear"] = detail::fromVector(motion.linear);
          movement["angular"] = detail::fromVector(motion.angular);
          break;
        case xgc2_math::scene::Motion::kPingPong:
          movement["point_a"] = detail::fromVector(motion.point_a);
          movement["point_b"] = detail::fromVector(motion.point_b);
          break;
        case xgc2_math::scene::Motion::kCircle:
          movement["center"] = detail::fromVector(motion.center);
          movement["phase"] = motion.phase;
          break;
      }
      obstacles.push_back(std::move(placed));
    }
  }
  const auto camera = yaml(extrinsic);
  if (!camera.is_object() || get(camera, "parent_frame") != frame)
    throw SceneError("cameraFrom extrinsic parent frame must be the world frame " + frame);
  auto name = strip(item(request, "name"));
  // The Kelvin sign is the only non-ASCII character whose Python lowercase
  // becomes ASCII; all other non-ASCII names fail the canonical ID grammar.
  for (auto at = name.find("\xe2\x84\xaa"); at != std::string::npos; at = name.find("\xe2\x84\xaa", at + 1))
    name.replace(at, 3, "k");
  for (auto& c : name) {
    if (c >= 'A' && c <= 'Z') c += 'a' - 'A';
    if (c == ' ' || c == '_') c = '-';
  }
  const auto result = document(Json{{"schema", kSchema}, {"id", name}, {"frame", frame}, {"obstacles", obstacles}});
  const auto encoded = dumpYaml(result);
  if (encoded.size() > kMaxDocumentBytes) throw SceneError("Assembled scene exceeds the supported size");
  return Json{{"sceneDocument", encoded}, {"task", task}};
}

Json assembleJson(const std::string& bytes) {
  if (bytes.size() > 4 * kMaxDocumentBytes) throw SceneError("Assembly input exceeds the supported size");
  const auto value = AssemblyInput(bytes).decode();
  // Keep Python's left-to-right argument lookup, including the first missing key.
  const auto& request = item(value, "request");
  const auto& documents = item(value, "documents");
  const auto& extrinsic = item(value, "extrinsic");
  return assemble(request, documents, extrinsic);
}

}  // namespace xgc_scene
