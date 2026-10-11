#include "xgc_scene/yaml_io.hpp"

#include "xgc_scene/json_io.hpp"
#include "scalars.hpp"

#include <openssl/sha.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <regex>
#include <sstream>

namespace xgc_scene {
namespace {

// These are PyYAML 5.3's YAML 1.1 resolvers, rather than yaml-cpp's conversions
// (notably, 1e3 is a string, while 1.0e+3 is a float).
std::string implicitTag(const std::string& value) {
  if (value.empty() || value == "~" || value == "null" || value == "Null" || value == "NULL") return "null";
  static const std::regex boolean("yes|Yes|YES|no|No|NO|true|True|TRUE|false|False|FALSE|on|On|ON|off|Off|OFF");
  static const std::regex integer(R"([-+]?0b[0-1_]+|[-+]?0[0-7_]+|[-+]?(0|[1-9][0-9_]*)|[-+]?0x[0-9a-fA-F_]+|[-+]?[1-9][0-9_]*(:[0-5]?[0-9])+)");
  static const std::regex floating(R"([-+]?[0-9][0-9_]*\.[0-9_]*([eE][-+][0-9]+)?|\.[0-9_]+([eE][-+][0-9]+)?|[-+]?[0-9][0-9_]*(:[0-5]?[0-9])+\.[0-9_]*|[-+]?\.(inf|Inf|INF)|\.(nan|NaN|NAN))");
  static const std::regex timestamp(R"([0-9]{4}-[0-9]{2}-[0-9]{2}|[0-9]{4}-[0-9]{1,2}-[0-9]{1,2}([Tt]|[ \t]+)[0-9]{1,2}:[0-9]{2}:[0-9]{2}(\.[0-9]*)?([ \t]*(Z|[-+][0-9]{1,2}(:[0-9]{2})?))?)");
  if (std::regex_match(value, boolean)) return "bool";
  const char first = value[0];
  if (first == '-' || first == '+' || first == '.' || (first >= '0' && first <= '9')) {
    if (std::regex_match(value, floating)) return "float";
    if (std::regex_match(value, integer)) return "int";
    if (std::regex_match(value, timestamp)) return "timestamp";
  }
  if (value == "<<") return "merge";
  if (value == "=") return "value";
  return "str";
}

std::string tag(const YAML::Node& node) {
  const auto& explicit_tag = node.Tag();
  if (explicit_tag == "!") return "str";
  if (explicit_tag.empty() || explicit_tag == "?") {
    if (node.IsScalar()) return implicitTag(node.Scalar());
    if (node.IsSequence()) return "seq";
    if (node.IsMap()) return "map";
    return "null";
  }
  const std::string prefix = "tag:yaml.org,2002:";
  if (explicit_tag.compare(0, prefix.size(), prefix) == 0) return explicit_tag.substr(prefix.size());
  throw SceneError("Invalid scene YAML: could not determine a constructor for the tag '" + explicit_tag + "'");
}

std::string scalarText(const YAML::Node& node, const std::string& bytes) {
  // yaml-cpp 0.6.2 decodes the YAML escapes \\N and \\_ as single
  // Latin-1 bytes. All other scalar text is UTF-8; repair those two escapes.
  const auto& value = node.Scalar();
  // Raw Unicode line/paragraph separators are YAML line breaks. yaml-cpp
  // retains the following indentation in a quoted scalar. Escaped separators
  // are ordinary scalar characters and must retain their following spaces.
  std::vector<bool> raw_breaks;
  auto at = static_cast<std::size_t>(node.Mark().pos);
  const char quote = at < bytes.size() && (bytes[at] == '\'' || bytes[at] == '"') ? bytes[at++] : 0;
  for (; quote && at < bytes.size(); ++at) {
    if (bytes[at] == quote) {
      if (quote == '\'' && at + 1 < bytes.size() && bytes[at + 1] == '\'') { ++at; continue; }
      break;
    }
    if (quote == '"' && bytes[at] == '\\' && at + 1 < bytes.size()) {
      const char escaped = bytes[++at];
      if (escaped == 'L' || escaped == 'P') raw_breaks.push_back(false);
      else if ((escaped == 'u' && (bytes.compare(at + 1, 4, "2028") == 0 || bytes.compare(at + 1, 4, "2029") == 0)) ||
               (escaped == 'U' && (bytes.compare(at + 1, 8, "00002028") == 0 || bytes.compare(at + 1, 8, "00002029") == 0)))
        raw_breaks.push_back(false);
    } else if (bytes.compare(at, 3, "\xe2\x80\xa8") == 0 || bytes.compare(at, 3, "\xe2\x80\xa9") == 0)
      raw_breaks.push_back(true);
  }
  std::string result;
  std::size_t break_index = 0;
  for (std::size_t i = 0; i < value.size();) {
    const auto byte = static_cast<unsigned char>(value[i]);
    if (byte == 0x85 || byte == 0xa0) { result += '\xc2'; result += value[i++]; }
    else {
      const auto count = byte < 0x80 ? 1u : byte < 0xe0 ? 2u : byte < 0xf0 ? 3u : 4u;
      const bool separator = value.compare(i, 3, "\xe2\x80\xa8") == 0 || value.compare(i, 3, "\xe2\x80\xa9") == 0;
      result.append(value, i, count);
      i += count;
      if (separator) {
        if (break_index < raw_breaks.size() && raw_breaks[break_index])
          while (i < value.size() && (value[i] == ' ' || value[i] == '\t')) ++i;
        ++break_index;
      }
    }
  }
  return result;
}

Json integerScalar(std::string value) {
  value.erase(std::remove(value.begin(), value.end(), '_'), value.end());
  const bool negative = !value.empty() && value[0] == '-';
  if (!value.empty() && (value[0] == '-' || value[0] == '+')) value.erase(0, 1);
  int base = 10;
  std::size_t begin = 0;
  if (value.compare(0, 2, "0b") == 0) { base = 2; begin = 2; }
  else if (value.compare(0, 2, "0x") == 0) { base = 16; begin = 2; }
  else if (value.size() > 1 && value[0] == '0') base = 8;
  std::uint64_t integer = 0;
  long double wide = 0;
  bool overflow = false;
  if (value.find(':') != std::string::npos && base == 10) {
    std::istringstream parts(value);
    std::string part;
    while (std::getline(parts, part, ':')) {
      const auto digit = std::stoull(part);
      wide = wide * 60 + digit;
      if (integer > (std::numeric_limits<std::uint64_t>::max() - digit) / 60) overflow = true;
      if (!overflow) integer = integer * 60 + digit;
    }
  } else {
    for (std::size_t i = begin; i < value.size(); ++i) {
      const auto c = value[i];
      const int digit = c >= '0' && c <= '9' ? c - '0' : c >= 'a' && c <= 'f' ? c - 'a' + 10 :
                        c >= 'A' && c <= 'F' ? c - 'A' + 10 : base;
      if (digit >= base) throw SceneError("Invalid scene YAML: invalid integer scalar");
      wide = wide * base + digit;
      if (integer > (std::numeric_limits<std::uint64_t>::max() - digit) / base) overflow = true;
      if (!overflow) integer = integer * base + digit;
    }
  }
  if (overflow) return static_cast<double>(negative ? -wide : wide);
  if (!negative) return integer;
  if (integer > (std::uint64_t{1} << 63)) return -static_cast<double>(integer);
  if (integer == (std::uint64_t{1} << 63)) return std::numeric_limits<std::int64_t>::min();
  return -static_cast<std::int64_t>(integer);
}

Json floatScalar(std::string value) {
  value.erase(std::remove(value.begin(), value.end(), '_'), value.end());
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) { return std::tolower(c); });
  const bool negative = !value.empty() && value[0] == '-';
  if (!value.empty() && (value[0] == '-' || value[0] == '+')) value.erase(0, 1);
  if (value == ".inf") return negative ? -std::numeric_limits<double>::infinity() : std::numeric_limits<double>::infinity();
  if (value == ".nan") return std::numeric_limits<double>::quiet_NaN();
  double result = 0;
  std::istringstream parts(value);
  std::vector<double> digits;
  std::string part;
  while (std::getline(parts, part, ':')) {
    char* end = nullptr;
    const double digit = std::strtod(part.c_str(), &end);
    if (end == part.c_str() || *end) throw SceneError("Invalid scene YAML: invalid float scalar");
    digits.push_back(digit);
  }
  // PyYAML sums sexagesimal digits from right to left.
  double base = 1;
  for (auto it = digits.rbegin(); it != digits.rend(); ++it) { result += *it * base; base *= 60; }
  return negative ? -result : result;
}

class SceneLoader {
  using Pairs = std::vector<std::pair<YAML::Node, YAML::Node>>;
  std::vector<YAML::Node> active_;
  const std::string& bytes_;

  void enter(const YAML::Node& node) {
    if (std::any_of(active_.begin(), active_.end(), [&](const auto& item) { return item.is(node); }))
      throw SceneError("Invalid scene YAML: found unconstructable recursive node");
    active_.push_back(node);
  }

  Pairs flatten(const YAML::Node& node) {
    enter(node);
    Pairs merged, own;
    for (const auto& pair : node) {
      if (tag(pair.first) != "merge") { own.emplace_back(pair.first, pair.second); continue; }
      const auto& source = pair.second;
      if (source.IsMap()) {
        auto items = flatten(source);
        merged.insert(merged.end(), items.begin(), items.end());
      } else if (source.IsSequence()) {
        std::vector<Pairs> maps;
        for (const auto& map : source) {
          if (!map.IsMap()) throw SceneError("Invalid scene YAML: expected a mapping for merging");
          maps.push_back(flatten(map));
        }
        for (auto it = maps.rbegin(); it != maps.rend(); ++it) merged.insert(merged.end(), it->begin(), it->end());
      } else throw SceneError("Invalid scene YAML: expected a mapping or list of mappings for merging");
    }
    active_.pop_back();
    merged.insert(merged.end(), own.begin(), own.end());
    return merged;
  }

 public:
  explicit SceneLoader(const std::string& bytes) : bytes_(bytes) {}
  Json construct(const YAML::Node& node) {
    if (node.IsNull()) return nullptr;
    const auto kind = tag(node);
    if (node.IsScalar()) {
      const auto value = scalarText(node, bytes_);
      if (kind == "str") return value;
      if (kind == "null") return nullptr;
      if (kind == "int") return integerScalar(value);
      if (kind == "float") return floatScalar(value);
      if (kind == "bool") {
        std::string lower = value;
        std::transform(lower.begin(), lower.end(), lower.begin(), [](unsigned char c) { return std::tolower(c); });
        if (lower == "true" || lower == "yes" || lower == "on") return true;
        if (lower == "false" || lower == "no" || lower == "off") return false;
        throw SceneError("Invalid scene YAML: invalid boolean scalar");
      }
    } else if (node.IsSequence() && kind == "seq") {
      enter(node);
      Json result = Json::array();
      for (const auto& item : node) result.push_back(construct(item));
      active_.pop_back();
      return result;
    } else if (node.IsMap() && kind == "map") {
      auto pairs = flatten(node);
      enter(node);
      std::vector<std::pair<Json, Json>> values;
      for (const auto& pair : pairs) {
        // Mapping names have to be strings even when the safe loader would
        // construct a date, set, sequence or another non-JSON key type.
        const auto key_tag = tag(pair.first);
        Json key;
        if (pair.first.IsScalar() && (key_tag == "str" || key_tag == "value")) key = scalarText(pair.first, bytes_);
        else if (key_tag == "timestamp" || key_tag == "binary" || key_tag == "set" || key_tag == "omap" || key_tag == "pairs") key = nullptr;
        else key = construct(pair.first);
        values.emplace_back(std::move(key), construct(pair.second));
      }
      active_.pop_back();
      return uniqueObject(values);
    }
    throw SceneError("Invalid scene YAML: unsupported scene document tag '" + node.Tag() + "'");
  }
};

bool lineBreak(unsigned c) { return c == '\n' || c == 0x85 || c == 0x2028 || c == 0x2029; }
bool yamlWhitespace(unsigned c) { return c == 0 || c == ' ' || c == '\t' || c == '\r' || lineBreak(c); }

class YamlEmitter {
  std::string out_;
  std::size_t column_ = 0;

  void write(const std::string& text) {
    out_ += text;
    column_ += detail::utf8Characters(text).size();
  }
  void indent(std::size_t count) {
    out_ += '\n';
    out_.append(count, ' ');
    column_ = count;
  }
  void startLine(std::size_t count) {
    out_.append(count, ' ');
    column_ = count;
  }
  char style(const std::string& text, bool key) {
    if (text.empty()) return '\'';
    const auto chars = detail::utf8Characters(text);
    bool plain = implicitTag(text) == "str";
    bool single = true;
    if (text.compare(0, 3, "---") == 0 || text.compare(0, 3, "...") == 0) plain = false;
    for (std::size_t i = 0; i < chars.size(); ++i) {
      const auto c = detail::codepoint(chars[i]);
      const auto previous = i == 0 ? 0 : detail::codepoint(chars[i - 1]);
      const bool followed = i + 1 == chars.size() || yamlWhitespace(detail::codepoint(chars[i + 1]));
      if (i == 0 && (std::string("#,[]{}&*!|>'\"%@`").find(static_cast<char>(c)) != std::string::npos && c < 128)) plain = false;
      if ((c == ':' && followed) || (i == 0 && (c == '?' || c == '-') && followed) ||
          (c == '#' && (i == 0 || yamlWhitespace(previous)))) plain = false;
      if ((i == 0 || i + 1 == chars.size()) && (c == ' ' || lineBreak(c))) plain = false;
      if (lineBreak(c)) { plain = false; if (key) single = false; }
      const bool special = !(c == '\n' || (c >= 0x20 && c <= 0x7e) || c == 0x2028 || c == 0x2029 ||
                             (c >= 0xa0 && c <= 0xd7ff) || (c >= 0xe000 && c <= 0xfffd)) || c == 0xfeff;
      if (special || (i > 0 && ((lineBreak(previous) && c == ' ') || (previous == ' ' && lineBreak(c)))))
        plain = single = false;
    }
    return plain ? 0 : single ? '\'' : '"';
  }

  std::string escape(unsigned c, const std::string& character) {
    switch (c) {
      case 0: return "\\0";
      case 7: return "\\a";
      case 8: return "\\b";
      case 9: return "\\t";
      case 10: return "\\n";
      case 11: return "\\v";
      case 12: return "\\f";
      case 13: return "\\r";
      case 27: return "\\e";
      case '"': case '\\': return "\\" + character;
      case 0x85: return "\\N";
      case 0xa0: return "\\_";
      case 0x2028: return "\\L";
      case 0x2029: return "\\P";
    }
    std::ostringstream result;
    result << (c <= 0xff ? "\\x" : c <= 0xffff ? "\\u" : "\\U") << std::hex << std::uppercase
           << std::setfill('0') << std::setw(c <= 0xff ? 2 : c <= 0xffff ? 4 : 8) << c;
    return result.str();
  }

  void string(const std::string& text, std::size_t continuation, bool key = false) {
    const char quote = style(text, key);
    if (quote) write(std::string(1, quote));
    const auto chars = detail::utf8Characters(text);
    if (quote == '"') {
      for (std::size_t i = 0; i < chars.size(); ++i) {
        const unsigned c = detail::codepoint(chars[i]);
        if (!key && c == ' ' && i > 0 && i + 1 < chars.size() && column_ > 80) {
          indent(continuation);
          if (chars[i + 1] == " ") write("\\");
          continue;
        }
        const bool escaped = c == '"' || c == '\\' || c == 0x85 || c == 0x2028 || c == 0x2029 || c == 0xfeff ||
          !((c >= 0x20 && c <= 0x7e) || (c >= 0xa0 && c <= 0xd7ff) || (c >= 0xe000 && c <= 0xfffd));
        write(escaped ? escape(c, chars[i]) : chars[i]);
      }
    } else {
      for (std::size_t i = 0; i < chars.size();) {
        const auto c = detail::codepoint(chars[i]);
        if (c == ' ') {
          const auto start = i;
          while (i < chars.size() && chars[i] == " ") ++i;
          if (!key && i - start == 1 && column_ > 80 && (quote == 0 || (start != 0 && i != chars.size())))
            indent(continuation);
          else for (auto j = start; j < i; ++j) write(" ");
        } else if (lineBreak(c)) {
          if (c == '\n') indent(0);
          while (i < chars.size() && lineBreak(detail::codepoint(chars[i]))) {
            if (chars[i] == "\n") indent(0);
            else { out_ += chars[i]; column_ = 0; }
            ++i;
          }
          out_.append(continuation, ' ');
          column_ = continuation;
        } else {
          write(chars[i++]);
          if (quote == '\'' && c == '\'') write("'");
        }
      }
    }
    if (quote) write(std::string(1, quote));
  }

  bool inlineValue(const Json& value) { return !value.is_structured() || value.empty(); }
  void scalar(const Json& value, std::size_t continuation) {
    if (value.is_string()) string(value.get_ref<const std::string&>(), continuation);
    else if (value.is_null()) write("null");
    else if (value.is_boolean()) write(value.get<bool>() ? "true" : "false");
    else if (value.is_number_float()) {
      std::string text = detail::pythonFloat(value.get<double>());
      if (text == "nan") text = ".nan";
      else if (text == "inf") text = ".inf";
      else if (text == "-inf") text = "-.inf";
      else if (text.find('.') == std::string::npos && text.find('e') != std::string::npos) text.insert(text.find('e'), ".0");
      write(text);
    } else write(value.dump());
  }

  void node(const Json& value, std::size_t level, bool first_inline = false) {
    if (inlineValue(value)) {
      if (!first_inline) startLine(level);
      scalar(value, first_inline ? level : level + 2);
      out_ += '\n'; column_ = 0;
    } else if (value.is_object()) {
      bool first = true;
      for (auto it = value.begin(); it != value.end(); ++it) {
        if (!first || !first_inline) startLine(level);
        first = false;
        const bool simple = detail::utf8Characters(it.key()).size() <= 128 && it.key().find('\n') == std::string::npos;
        if (!simple) write("? ");
        string(it.key(), level + 2, simple);
        if (!simple) { indent(level); write(":"); } else write(":");
        if (inlineValue(it.value())) {
          write(" "); scalar(it.value(), level + 2); out_ += '\n'; column_ = 0;
        } else {
          out_ += '\n'; column_ = 0;
          node(it.value(), level + (it.value().is_array() ? 0 : 2));
        }
      }
    } else {
      bool first = true;
      for (const auto& item : value) {
        if (!first || !first_inline) startLine(level);
        first = false;
        write("- ");
        node(item, level + 2, true);
      }
    }
  }

 public:
  std::string dump(const Json& value) { node(value, 0); return std::move(out_); }
};

}  // namespace

Json parseYaml(const std::string& bytes) {
  try {
    const auto nodes = YAML::LoadAll(bytes);
    if (nodes.size() > 1) throw SceneError("Invalid scene YAML: expected a single document in the stream");
    return nodes.empty() ? Json(nullptr) : SceneLoader{bytes}.construct(nodes[0]);
  } catch (const YAML::Exception& error) {
    throw SceneError("Invalid scene YAML: " + std::string(error.what()));
  }
}

std::string dumpYaml(const Json& value) { return YamlEmitter{}.dump(value); }

std::string digest(const std::string& bytes) {
  unsigned char hash[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char*>(bytes.data()), bytes.size(), hash);
  std::ostringstream out;
  out << std::hex << std::setfill('0');
  for (unsigned char byte : hash) out << std::setw(2) << static_cast<unsigned>(byte);
  return out.str();
}

}  // namespace xgc_scene
