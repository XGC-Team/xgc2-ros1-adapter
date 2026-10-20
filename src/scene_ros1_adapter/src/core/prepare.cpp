#include "xgc_scene/prepare.hpp"

#include "xgc_scene/document.hpp"
#include "xgc_scene/json_io.hpp"
#include "xgc_scene/yaml_io.hpp"
#include "scalars.hpp"

#include <openssl/sha.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <fcntl.h>
#include <fstream>
#include <functional>
#include <iomanip>
#include <map>
#include <optional>
#include <regex>
#include <set>
#include <sstream>
#include <system_error>
#include <sys/stat.h>
#include <unistd.h>

namespace xgc_scene {
namespace {
namespace fs = std::filesystem;

bool truth(const Json& value) {
  if (value.is_null()) return false;
  if (value.is_boolean()) return value.get<bool>();
  if (value.is_number()) return value.get<double>() != 0;
  if (value.is_string()) return !value.get_ref<const std::string&>().empty();
  return !value.empty();
}

bool finite(const Json& value) { return value.is_number() && std::isfinite(value.get<double>()); }

bool shortIdentity(const Json& value) {
  static const std::regex pattern("[A-Za-z0-9_-]{1,128}");
  return value.is_string() && std::regex_match(value.get_ref<const std::string&>(), pattern);
}

bool trimmed(const std::string& text) {
  const auto characters = detail::utf8Characters(text);
  return !characters.empty() && !detail::pythonWhitespace(detail::codepoint(characters.front())) &&
         !detail::pythonWhitespace(detail::codepoint(characters.back()));
}

fs::path providerPath(const fs::path& path) {
  // pathlib removes trailing separators before checking whether a grant is a link.
  auto result = path;
  while (result != result.root_path() && result.filename().empty()) result = result.parent_path();
  return result;
}

std::string normalPath(const std::string& path) {
  std::vector<std::string> parts;
  for (std::size_t at = 0; at < path.size();) {
    const auto end = path.find('/', at);
    const auto part = path.substr(at, end == std::string::npos ? end : end - at);
    if (part == "..") { if (!parts.empty()) parts.pop_back(); }
    else if (!part.empty() && part != ".") parts.push_back(part);
    if (end == std::string::npos) break;
    at = end + 1;
  }
  std::string result = path.compare(0, 2, "//") == 0 && path.compare(0, 3, "///") != 0 ? "//" : "/";
  for (const auto& part : parts) { if (result.back() != '/') result += '/'; result += part; }
  return result;
}

Json manifestAt(const fs::path& directory) {
  const auto result = parseJson(read(directory / "manifest.json", 65536));
  if (!result.is_object() || !result.value("schemaVersion", Json()).is_number_integer() ||
      result.at("schemaVersion") != 1 || result.value("kind", Json()) != "xgc.scene-replay-asset.v1")
    throw SceneError("Unsupported scene asset manifest schema");
  return result;
}

// The Session froze the revision it selected. Assets sealed before revisions
// existed carry none, which freezes (and compares) as the empty string.
void expectRevision(const Json& manifest, const Json& expected) {
  if (expected.is_null()) return;
  if (!expected.is_string()) throw SceneError("The expected scene asset revision must be a string");
  const auto actual = manifest.value("revisionId", Json(""));
  if (!actual.is_string() || actual != expected)
    throw SceneError("Scene asset revision conflicts with the requested reference");
}

void workspaceChildren(const fs::path& grant, const fs::path& output) {
  const auto root = fs::canonical(grant);
  const auto relative = output.lexically_relative(root);
  if (relative.empty() || (!relative.empty() && *relative.begin() == ".."))
    throw SceneError("Resource path is outside the granted root");
  auto current = root;
  for (const auto& part : relative) {
    if (part == ".") continue;
    current /= part;
    if (fs::is_symlink(current)) throw SceneError("Provider workspace cannot contain symlinks");
    if (::mkdir(current.c_str(), 0750) != 0 && (errno != EEXIST || !fs::is_directory(current)))
      throw std::system_error(errno, std::generic_category(), current.string());
  }
}

struct Stage {
  fs::path path;
  explicit Stage(const fs::path& output) {
    const auto pattern = (output / ".scene-prepare-XXXXXX").string();
    std::vector<char> name(pattern.begin(), pattern.end());
    name.push_back('\0');
    if (!::mkdtemp(name.data())) throw std::system_error(errno, std::generic_category(), pattern);
    path = name.data();
  }
  ~Stage() { std::error_code error; fs::remove_all(path, error); }
};

// A flat tree and explicit parse/write stacks bound memory to the input and
// avoid recursion on deeply nested worlds. Comments and processing instructions
// are discarded, just as ElementTree's default TreeBuilder discards them.
struct Element {
  std::string tag, text, tail;
  std::vector<std::pair<std::string, std::string>> attributes;
  std::vector<std::size_t> children;
};

[[noreturn]] void invalidXml() { throw SceneError("Invalid Gazebo world XML"); }
bool xmlSpace(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; }

std::string xmlCharacter(unsigned c) {
  if (!(c == 9 || c == 10 || c == 13 || (c >= 0x20 && c <= 0xd7ff) ||
        (c >= 0xe000 && c <= 0xfffd) || (c >= 0x10000 && c <= 0x10ffff))) invalidXml();
  std::string result;
  if (c < 0x80) result += static_cast<char>(c);
  else if (c < 0x800) {
    result += static_cast<char>(0xc0 | (c >> 6)); result += static_cast<char>(0x80 | (c & 0x3f));
  } else if (c < 0x10000) {
    result += static_cast<char>(0xe0 | (c >> 12)); result += static_cast<char>(0x80 | ((c >> 6) & 0x3f));
    result += static_cast<char>(0x80 | (c & 0x3f));
  } else {
    result += static_cast<char>(0xf0 | (c >> 18)); result += static_cast<char>(0x80 | ((c >> 12) & 0x3f));
    result += static_cast<char>(0x80 | ((c >> 6) & 0x3f)); result += static_cast<char>(0x80 | (c & 0x3f));
  }
  return result;
}

std::string xmlText(const std::string& text, bool attribute = false, bool entities = true) {
  std::string result;
  for (std::size_t i = 0; i < text.size(); ++i) {
    if (entities && text[i] == '&') {
      const auto end = text.find(';', i + 1);
      if (end == std::string::npos) invalidXml();
      const auto name = text.substr(i + 1, end - i - 1);
      if (name == "amp") result += '&';
      else if (name == "lt") result += '<';
      else if (name == "gt") result += '>';
      else if (name == "quot") result += '"';
      else if (name == "apos") result += '\'';
      else if (!name.empty() && name[0] == '#') {
        const bool hex = name.size() > 1 && name[1] == 'x';
        const auto digits = name.substr(hex ? 2 : 1);
        if (digits.empty()) invalidXml();
        unsigned code = 0;
        for (const auto c : digits) {
          const unsigned digit = c >= '0' && c <= '9' ? c - '0' :
              hex && c >= 'a' && c <= 'f' ? c - 'a' + 10 :
              hex && c >= 'A' && c <= 'F' ? c - 'A' + 10 : 16;
          const unsigned base = hex ? 16 : 10;
          if (digit >= base || code > (0x10ffff - digit) / base) invalidXml();
          code = code * base + digit;
        }
        result += xmlCharacter(code);
      } else invalidXml();
      i = end;
    } else if (text[i] == '\r') {
      if (i + 1 < text.size() && text[i + 1] == '\n') ++i;
      result += attribute ? ' ' : '\n';
    } else if (attribute && (text[i] == '\n' || text[i] == '\t')) result += ' ';
    else result += text[i];
  }
  return result;
}

std::string escapeXml(const std::string& text, bool attribute = false) {
  std::string result;
  for (const auto c : text) {
    if (c == '&') result += "&amp;";
    else if (c == '<') result += "&lt;";
    else if (c == '>') result += "&gt;";
    else if (attribute && c == '"') result += "&quot;";
    else if (attribute && c == '\r') result += "&#13;";
    else if (attribute && c == '\n') result += "&#10;";
    else if (attribute && c == '\t') result += "&#09;";
    else result += c;
  }
  return result;
}

class WorldXml {
 public:
  std::vector<Element> elements;

  explicit WorldXml(const std::string& raw) {
    // Validate UTF-8 and XML characters before scanning markup.
    for (std::size_t i = 0; i < raw.size();) {
      const auto first = static_cast<unsigned char>(raw[i]);
      const std::size_t count = first < 0x80 ? 1 : first >= 0xc2 && first <= 0xdf ? 2 :
          first >= 0xe0 && first <= 0xef ? 3 : first >= 0xf0 && first <= 0xf4 ? 4 : 0;
      if (!count || i + count > raw.size()) invalidXml();
      for (std::size_t j = 1; j < count; ++j)
        if ((static_cast<unsigned char>(raw[i + j]) & 0xc0) != 0x80) invalidXml();
      const auto character = raw.substr(i, count);
      if (xmlCharacter(detail::codepoint(character)) != character) invalidXml();
      i += count;
    }
    std::size_t at = raw.compare(0, 3, "\xef\xbb\xbf") == 0 ? 3 : 0;
    std::vector<std::size_t> stack;
    std::vector<std::string> names;
    std::map<std::string, std::string> namespaces{{"xml", "http://www.w3.org/XML/1998/namespace"}};
    using Binding = std::pair<std::string, std::optional<std::string>>;
    std::vector<std::vector<Binding>> scopes;
    const auto restore = [&](const std::vector<Binding>& bindings) {
      for (auto it = bindings.rbegin(); it != bindings.rend(); ++it) {
        if (it->second) namespaces[it->first] = *it->second;
        else namespaces.erase(it->first);
      }
    };
    const auto space = [&] { while (at < raw.size() && xmlSpace(raw[at])) ++at; };
    const auto name = [&]() {
      const auto start = at;
      while (at < raw.size()) {
        const auto c = static_cast<unsigned char>(raw[at]);
        const bool letter = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || c >= 0x80;
        if (!(letter || c == '_' || c == ':' || (at > start && ((c >= '0' && c <= '9') || c == '-' || c == '.')))) break;
        ++at;
      }
      if (at == start) invalidXml();
      return raw.substr(start, at - start);
    };
    const auto append = [&](const std::string& text) {
      if (stack.empty()) {
        if (!std::all_of(text.begin(), text.end(), xmlSpace)) invalidXml();
      } else {
        auto& element = elements[stack.back()];
        if (element.children.empty()) element.text += text;
        else elements[element.children.back()].tail += text;
      }
    };
    while (at < raw.size()) {
      if (raw[at] != '<') {
        const auto end = raw.find('<', at);
        const auto text = raw.substr(at, end == std::string::npos ? end : end - at);
        if (text.find("]]>") != std::string::npos) invalidXml();
        append(xmlText(text)); at = end == std::string::npos ? raw.size() : end;
      } else if (raw.compare(at, 4, "<!--") == 0) {
        const auto end = raw.find("-->", at + 4);
        if (end == std::string::npos || raw.substr(at + 4, end - at - 4).find("--") != std::string::npos) invalidXml();
        at = end + 3;
      } else if (raw.compare(at, 2, "<?") == 0) {
        const auto start = at;
        at += 2;
        const auto instruction = name();
        const auto end = raw.find("?>", at);
        if (end == std::string::npos) invalidXml();
        auto reserved = instruction;
        std::transform(reserved.begin(), reserved.end(), reserved.begin(), [](unsigned char c) {
          return c >= 'A' && c <= 'Z' ? c - 'A' + 'a' : c;
        });
        if (reserved == "xml") {
          if (instruction != "xml" || start != (raw.compare(0, 3, "\xef\xbb\xbf") == 0 ? 3u : 0u) || !elements.empty()) invalidXml();
          std::vector<std::pair<std::string, std::string>> declaration;
          while (at < end) {
            const auto before = at; space();
            if (at == end) break;
            if (at == before || declaration.size() == 3) invalidXml();
            const auto key = name(); space();
            if (at >= end || raw[at++] != '=') invalidXml();
            space();
            if (at >= end || (raw[at] != '\'' && raw[at] != '"')) invalidXml();
            const auto quote = raw[at++];
            const auto finish = raw.find(quote, at);
            if (finish == std::string::npos || finish >= end) invalidXml();
            declaration.emplace_back(key, raw.substr(at, finish - at)); at = finish + 1;
          }
          if (declaration.empty() || declaration[0].first != "version" ||
              declaration[0].second.compare(0, 2, "1.") != 0 || declaration[0].second.size() < 3 ||
              declaration[0].second.find_first_not_of("0123456789", 2) != std::string::npos) invalidXml();
          std::size_t index = 1;
          if (index < declaration.size() && declaration[index].first == "encoding") {
            const auto& encoding = declaration[index++].second;
            if (encoding.empty() || !((encoding[0] >= 'A' && encoding[0] <= 'Z') || (encoding[0] >= 'a' && encoding[0] <= 'z')) ||
                encoding.find_first_not_of("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789._-") != std::string::npos) invalidXml();
          }
          if (index < declaration.size() && declaration[index].first == "standalone") {
            const auto value = declaration[index++].second;
            if (value != "yes" && value != "no") invalidXml();
          }
          if (index != declaration.size()) invalidXml();
        } else if (at != end && !xmlSpace(raw[at])) invalidXml();
        at = end + 2;
      } else if (raw.compare(at, 9, "<![CDATA[") == 0) {
        const auto end = raw.find("]]>", at + 9);
        if (stack.empty() || end == std::string::npos) invalidXml();
        append(xmlText(raw.substr(at + 9, end - at - 9), false, false)); at = end + 3;
      } else if (raw.compare(at, 2, "</") == 0) {
        at += 2;
        const auto closing = name(); space();
        if (stack.empty() || names.back() != closing || at >= raw.size() || raw[at++] != '>') invalidXml();
        stack.pop_back(); names.pop_back(); restore(scopes.back()); scopes.pop_back();
      } else {
        if (raw.compare(at, 2, "<!") == 0 || (stack.empty() && !elements.empty())) invalidXml();
        ++at;
        Element element;
        const auto opening = name();
        std::vector<Binding> scope;
        std::vector<std::pair<std::string, std::string>> attributes;
        std::set<std::string> keys;
        bool empty = false;
        while (true) {
          const auto before = at; space();
          if (at >= raw.size()) invalidXml();
          if (raw[at] == '>') { ++at; break; }
          if (raw.compare(at, 2, "/>") == 0) { at += 2; empty = true; break; }
          if (at == before) invalidXml();
          const auto key = name(); space();
          if (!keys.insert(key).second || at >= raw.size() || raw[at++] != '=') invalidXml();
          space();
          if (at >= raw.size() || (raw[at] != '\'' && raw[at] != '"')) invalidXml();
          const auto quote = raw[at++];
          const auto end = raw.find(quote, at);
          if (end == std::string::npos || raw.substr(at, end - at).find('<') != std::string::npos) invalidXml();
          const auto value = xmlText(raw.substr(at, end - at), true); at = end + 1;
          if (key == "xmlns" || key.compare(0, 6, "xmlns:") == 0) {
            const auto prefix = key == "xmlns" ? "" : key.substr(6);
            if (prefix == "xmlns" || (!prefix.empty() && value.empty()) ||
                (prefix == "xml") != (value == "http://www.w3.org/XML/1998/namespace") ||
                value == "http://www.w3.org/2000/xmlns/") invalidXml();
            const auto old = namespaces.find(prefix);
            scope.emplace_back(prefix, old == namespaces.end() ? std::optional<std::string>{} : old->second);
            namespaces[prefix] = value;
          } else attributes.emplace_back(key, value);
        }
        const auto expanded = [&](const std::string& key, bool attribute) {
          const auto colon = key.find(':');
          if (colon != std::string::npos && (colon == 0 || key.find(':', colon + 1) != std::string::npos)) invalidXml();
          if (colon == std::string::npos && attribute) return key;
          const auto found = namespaces.find(colon == std::string::npos ? "" : key.substr(0, colon));
          if (found == namespaces.end()) { if (colon != std::string::npos) invalidXml(); return key; }
          return found->second.empty() ? key : "{" + found->second + "}" + key.substr(colon == std::string::npos ? 0 : colon + 1);
        };
        element.tag = expanded(opening, false);
        keys.clear();
        for (const auto& attribute : attributes) {
          const auto key = expanded(attribute.first, true);
          if (!keys.insert(key).second) invalidXml();
          element.attributes.emplace_back(key, attribute.second);
        }
        const auto index = elements.size();
        elements.push_back(std::move(element));
        if (!stack.empty()) elements[stack.back()].children.push_back(index);
        if (!empty) { stack.push_back(index); names.push_back(opening); scopes.push_back(std::move(scope)); }
        else restore(scope);
      }
    }
    if (elements.empty() || !stack.empty()) invalidXml();
  }

  std::vector<std::size_t> findAll(std::size_t parent, const std::string& tag) const {
    std::vector<std::size_t> result;
    for (const auto child : elements[parent].children) if (elements[child].tag == tag) result.push_back(child);
    return result;
  }
  std::size_t add(std::size_t parent, const std::string& tag, const std::string& text = "",
                  const std::vector<std::pair<std::string, std::string>>& attributes = {}) {
    const auto index = elements.size();
    elements.push_back({tag, text, "", attributes, {}});
    elements[parent].children.push_back(index);
    return index;
  }
  std::string attribute(std::size_t index, const std::string& key) const {
    for (const auto& value : elements[index].attributes) if (value.first == key) return value.second;
    return {};
  }
  std::string encode() const {
    std::map<std::string, std::string> namespaces;
    const auto qualified = [&](const std::string& tag) {
      if (tag.empty() || tag[0] != '{') return tag;
      const auto end = tag.find('}');
      const auto uri = tag.substr(1, end - 1);
      auto found = namespaces.find(uri);
      if (found == namespaces.end()) {
        static const std::map<std::string, std::string> registered{
            {"http://www.w3.org/XML/1998/namespace", "xml"}, {"http://www.w3.org/1999/xhtml", "html"},
            {"http://www.w3.org/1999/02/22-rdf-syntax-ns#", "rdf"}, {"http://schemas.xmlsoap.org/wsdl/", "wsdl"},
            {"http://www.w3.org/2001/XMLSchema", "xs"}, {"http://www.w3.org/2001/XMLSchema-instance", "xsi"},
            {"http://purl.org/dc/elements/1.1/", "dc"}};
        const auto known = registered.find(uri);
        const auto prefix = known == registered.end() ? "ns" + std::to_string(namespaces.size()) : known->second;
        found = namespaces.emplace(uri, prefix).first;
      }
      return found->second + ":" + tag.substr(end + 1);
    };
    for (const auto& element : elements) {
      qualified(element.tag);
      for (const auto& attribute : element.attributes) qualified(attribute.first);
    }
    std::string result = "<?xml version='1.0' encoding='utf-8'?>\n";
    std::vector<std::pair<std::size_t, bool>> stack{{0, false}};
    while (!stack.empty()) {
      const auto entry = stack.back(); stack.pop_back();
      const auto& element = elements[entry.first];
      const auto tag = qualified(element.tag);
      if (entry.second) { result += "</" + tag + ">" + escapeXml(element.tail); continue; }
      result += "<" + tag;
      if (entry.first == 0) {
        std::map<std::string, std::string> sorted;
        for (const auto& ns : namespaces) if (ns.second != "xml") sorted.emplace(ns.second, ns.first);
        for (const auto& ns : sorted) result += " xmlns:" + ns.first + "=\"" + escapeXml(ns.second, true) + "\"";
      }
      for (const auto& attribute : element.attributes)
        result += " " + qualified(attribute.first) + "=\"" + escapeXml(attribute.second, true) + "\"";
      if (element.text.empty() && element.children.empty()) result += " />" + escapeXml(element.tail);
      else {
        result += ">" + escapeXml(element.text);
        stack.emplace_back(entry.first, true);
        for (auto it = element.children.rbegin(); it != element.children.rend(); ++it) stack.emplace_back(*it, false);
      }
    }
    return result;
  }
};

}  // namespace

std::string read(const fs::path& path, std::size_t limit) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) throw std::system_error(errno, std::generic_category(), path.string());
  std::string data;
  std::array<char, 65536> block;
  while (data.size() <= limit && stream) {
    stream.read(block.data(), std::min(block.size(), limit - data.size() + 1));
    data.append(block.data(), static_cast<std::size_t>(stream.gcount()));
  }
  if (stream.bad()) throw std::system_error(errno, std::generic_category(), path.string());
  if (data.empty() || data.size() > limit)
    throw SceneError("Input " + path.string() + " exceeds its " + std::to_string(limit) + " byte limit or is empty");
  return data;
}

fs::path checkedPath(const Json& value, const fs::path& root, bool directory) {
  if (!value.is_string() || !fs::path(value.get<std::string>()).is_absolute())
    throw SceneError("An absolute provider resource path is required");
  const auto path = fs::canonical(value.get<std::string>());
  const auto ends = std::mismatch(root.begin(), root.end(), path.begin(), path.end());
  if (ends.first != root.end()) throw SceneError("Resource path is outside the granted root");
  if (directory && !fs::is_directory(path)) throw SceneError("Resource directory is required");
  if (!directory && !fs::is_regular_file(path)) throw SceneError("Resource file is required");
  return path;
}

std::string sha256(const fs::path& path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) throw std::system_error(errno, std::generic_category(), path.string());
  SHA256_CTX context;
  SHA256_Init(&context);
  std::array<char, 65536> block;
  while (stream) {
    stream.read(block.data(), block.size());
    SHA256_Update(&context, block.data(), static_cast<std::size_t>(stream.gcount()));
  }
  if (stream.bad()) throw std::system_error(errno, std::generic_category(), path.string());
  std::array<unsigned char, SHA256_DIGEST_LENGTH> hash;
  SHA256_Final(hash.data(), &context);
  std::ostringstream result;
  result << std::hex << std::setfill('0');
  for (const auto byte : hash) result << std::setw(2) << static_cast<unsigned>(byte);
  return result.str();
}

std::string attachment(const fs::path& directory, const Json& value, bool required) {
  if (value.is_null() || (value.is_object() && value.empty()) ||
      value == Json{{"file", ""}, {"sha256", ""}}) {
    if (required) throw SceneError("Required scene attachment is absent");
    return {};
  }
  if (!value.is_object()) throw SceneError("Scene attachment must be an object");
  const auto filename = value.value("file", Json());
  if (!filename.is_string()) throw SceneError("Scene attachment must use a plain asset filename");
  const auto name = filename.get<std::string>();
  if (name.empty() || fs::path(name).filename() != name || name[0] == '.' || name.find("..") != std::string::npos)
    throw SceneError("Scene attachment must use a plain asset filename");
  const auto path = checkedPath((directory / name).string(), directory);
  const auto expected = value.value("sha256", Json());
  if (!expected.is_string() || expected.get_ref<const std::string&>().size() != 64 ||
      expected.get_ref<const std::string&>().find_first_not_of("0123456789abcdef") != std::string::npos)
    throw SceneError("Scene attachment requires its exact SHA256");
  if (sha256(path) != expected) throw SceneError("Scene attachment content does not match its manifest SHA256");
  return path.string();
}

Json parameters(const Json& value, bool template_world) {
  if (value.contains("chassis_robot_ids"))
    throw SceneError("scene parameter chassis_robot_ids is no longer accepted; the Gazebo chassis HOLD roster is dynamic");
  auto world_parameters = value;
  world_parameters.erase("lidar");
  fields(world_parameters, {"overrideWorldPhysicsTiming", "maxStepSize", "realTimeUpdateRate", "generation",
                 "required_components", "clock_rate_hz"});
  Json result{{"overrideWorldPhysicsTiming", template_world}, {"maxStepSize", .004}, {"realTimeUpdateRate", 250}};
  result.update(world_parameters);
  const auto rate = result.value("clock_rate_hz", Json(0));
  if (!finite(rate) || rate.get<double>() < 0 || rate.get<double>() > 1e6)
    throw SceneError("clock_rate_hz must be finite in [0, 1000000]");
  result["clock_rate_hz"] = rate;
  if (!result.at("overrideWorldPhysicsTiming").is_boolean())
    throw SceneError("overrideWorldPhysicsTiming must be a boolean");
  for (const auto& name : {"maxStepSize", "realTimeUpdateRate"}) {
    const auto& number = result.at(name);
    const bool positive = std::string(name) == "maxStepSize";
    if (!finite(number) || (positive ? number.get<double>() <= 0 : number.get<double>() < 0))
      throw SceneError(std::string(name) + " must be a finite " + (positive ? "positive" : "nonnegative") + " number");
  }
  const auto generation = result.value("generation", Json::object());
  if (!generation.is_object() || std::any_of(generation.begin(), generation.end(), [](const auto& item) { return !finite(item); }))
    throw SceneError("generation must contain finite numeric parameters");
  static const std::regex identity("[A-Za-z0-9][A-Za-z0-9_.-]{0,63}");
  const auto roster = result.value("required_components", Json::array({"vrpn"}));
  bool valid = roster.is_array() && roster.size() <= 16;
  std::set<std::string> unique;
  if (valid) for (const auto& item : roster) {
    if (!item.is_string() || !std::regex_match(item.get_ref<const std::string&>(), identity) ||
        !unique.insert(item.get<std::string>()).second) { valid = false; break; }
  }
  if (!valid) throw SceneError("required_components must contain at most 16 unique ASCII short identifiers");
  result["required_components"] = roster;
  return result;
}

void atomic(const fs::path& path, const std::string& data) {
  const auto pattern = (path.parent_path() / ("." + path.filename().string() + ".XXXXXX")).string();
  std::vector<char> name(pattern.begin(), pattern.end()); name.push_back('\0');
  int file = ::mkstemp(name.data());
  if (file < 0) throw std::system_error(errno, std::generic_category(), pattern);
  int directory = -1;
  try {
    for (std::size_t written = 0; written < data.size();) {
      const auto count = ::write(file, data.data() + written, data.size() - written);
      if (count < 0 && errno == EINTR) continue;
      if (count <= 0) throw std::system_error(count == 0 ? EIO : errno, std::generic_category(), path.string());
      written += static_cast<std::size_t>(count);
    }
    if (::fsync(file) != 0) throw std::system_error(errno, std::generic_category(), path.string());
    const auto closed = ::close(file); file = -1;
    if (closed != 0 || ::rename(name.data(), path.c_str()) != 0)
      throw std::system_error(errno, std::generic_category(), path.string());
    directory = ::open(path.parent_path().c_str(), O_RDONLY | O_DIRECTORY);
    if (directory < 0 || ::fsync(directory) != 0)
      throw std::system_error(errno, std::generic_category(), path.parent_path().string());
    ::close(directory);
  } catch (...) {
    if (file >= 0) ::close(file);
    if (directory >= 0) ::close(directory);
    ::unlink(name.data());
    throw;
  }
}

Json prepare(const Json& request) {
  fields(request, {"schema", "configuration_revision", "asset_directory", "world_file", "resource_root",
                   "socket_path", "target_id", "output_grant", "parameters", "editable", "run_id", "expected_revision_id"},
                  {"schema", "configuration_revision", "resource_root", "socket_path", "target_id", "output_grant"});
  const auto& revision = request.at("configuration_revision");
  if (request.at("schema") != kPrepareSchema || !revision.is_number_integer() ||
      revision.get<double>() < 1 || revision.get<double>() > 9007199254740991.0)
    throw SceneError("A versioned simulation prepare request is required");
  if (truth(request.value("asset_directory", Json())) == truth(request.value("world_file", Json())))
    throw SceneError("Exactly one asset_directory or world_file is required");
  const auto resource = request.at("resource_root");
  if (!resource.is_string() || !fs::path(resource.get<std::string>()).is_absolute() || !fs::is_directory(resource.get<std::string>()))
    throw SceneError("An existing absolute resource_root is required");
  const auto root = fs::canonical(resource.get<std::string>());
  const auto& grant = fields(request.at("output_grant"), {"directory", "max_bytes"}, {"directory", "max_bytes"});
  const auto output_value = grant.at("directory");
  if (!output_value.is_string()) throw SceneError("The output grant must identify an existing absolute directory");
  auto output = providerPath(output_value.get<std::string>());
  if (!output.is_absolute() || !fs::is_directory(output) || fs::is_symlink(output))
    throw SceneError("The output grant must identify an existing absolute directory");
  output = fs::canonical(output);
  const auto& max_bytes = grant.at("max_bytes");
  if (!max_bytes.is_number_integer() || max_bytes.get<double>() < 1 || max_bytes.get<double>() > 64 * 1024 * 1024)
    throw SceneError("The output grant requires a byte limit of at most 64 MiB");
  const auto limit = max_bytes.get<std::size_t>();
  const auto& socket = request.at("socket_path");
  if (!socket.is_string() || !fs::path(socket.get<std::string>()).is_absolute() ||
      normalPath(socket.get<std::string>()) != socket || socket.get_ref<const std::string&>().size() > 107)
    throw SceneError("A canonical absolute world socket path is required");
  const auto& target = request.at("target_id");
  if (!target.is_string() || !trimmed(target.get<std::string>()) || detail::utf8Characters(target.get<std::string>()).size() > 128)
    throw SceneError("An explicit target_id is required");
  const auto editable = request.value("editable", Json(false));
  if (!editable.is_boolean()) throw SceneError("editable must be a boolean");
  Json manifest = Json::object();
  std::string scene_file, media_file, camera_file, extrinsic_file, asset_directory;
  bool project_geometry = false, frozen = true, template_world = false;
  fs::path world;
  if (truth(request.value("asset_directory", Json()))) {
    const auto directory = checkedPath(request.at("asset_directory"), root, true);
    asset_directory = directory.string();
    manifest = manifestAt(directory);
    expectRevision(manifest, request.value("expected_revision_id", Json()));
    const auto simulators = manifest.value("simulators", Json::object());
    if (!simulators.is_object() || !simulators.value("gazebo", Json()).is_object())
      throw SceneError("The asset must declare Gazebo geometry support");
    for (const auto& support : simulators)
      if (!support.is_object() || support.value("geometry", Json()) != "document")
        throw SceneError("Scene asset geometry must be document");
    project_geometry = true;
    scene_file = attachment(directory, manifest.value("sceneDocument", Json()), true);
    if (truth(manifest.value("gazeboWorld", Json()))) world = attachment(directory, manifest.at("gazeboWorld"), true);
    else {
      world = kDefaultWorld;
      if (!fs::is_regular_file(world)) throw SceneError("The provider editable-world template is not installed");
      template_world = true;
    }
    frozen = !editable.get<bool>();
    media_file = attachment(directory, manifest.value("media", Json()));
    camera_file = attachment(directory, manifest.value("cameraInfo", Json()));
    extrinsic_file = attachment(directory, manifest.value("extrinsic", Json()));
  } else world = checkedPath(request.at("world_file"), root);
  const auto physics = parameters(request.value("parameters", Json::object()), template_world);
  const auto raw = read(world, kMaxDocumentBytes);
  auto upper = raw;
  std::transform(upper.begin(), upper.end(), upper.begin(), [](unsigned char c) { return c >= 'a' && c <= 'z' ? c - 'a' + 'A' : c; });
  if (upper.find("<!DOCTYPE") != std::string::npos || upper.find("<!ENTITY") != std::string::npos)
    throw SceneError("World assets cannot declare XML entities");
  WorldXml sdf(raw);
  const auto worlds = sdf.findAll(0, "world");
  if (sdf.elements[0].tag != "sdf" || worlds.size() != 1) throw SceneError("A single-world SDF asset is required");
  const auto live = worlds[0];
  for (const auto plugin : sdf.findAll(live, "plugin")) {
    const auto filename = sdf.attribute(plugin, "filename");
    if (filename == "libxgc2_simulation_world.so" || filename == "libxgc2_scene_authoring_world.so")
      throw SceneError("The input world already embeds a platform control owner");
  }
  if (physics.at("overrideWorldPhysicsTiming").get<bool>()) {
    const auto found = sdf.findAll(live, "physics");
    const auto element = found.empty() ? sdf.add(live, "physics", "", {{"name", "simulation_physics"}, {"type", "ode"}}) : found[0];
    for (const auto& tag : {std::make_pair("max_step_size", "maxStepSize"), std::make_pair("real_time_update_rate", "realTimeUpdateRate")}) {
      const auto items = sdf.findAll(element, tag.first);
      const auto item = items.empty() ? sdf.add(element, tag.first) : items[0];
      sdf.elements[item].text = dumpJson(physics.at(tag.second));
    }
  }
  const auto plugin = sdf.add(live, "plugin", "", {{"name", "xgc2_simulation"}, {"filename", "libxgc2_simulation_world.so"}});
  sdf.add(plugin, "socket_path", socket.get<std::string>());
  sdf.add(plugin, "target_id", target.get<std::string>());
  sdf.add(plugin, "resource_root", root.string());
  sdf.add(plugin, "configuration_revision", dumpJson(revision));
  for (const auto& identity : physics.at("required_components")) sdf.add(plugin, "required_component", identity.get<std::string>());
  sdf.add(plugin, "clock_rate_hz", dumpJson(physics.at("clock_rate_hz")));
  const auto encoded = sdf.encode();
  std::vector<std::pair<fs::path, std::string>> files{{output / "prepared-world.world", encoded}};
  std::string working_file;
  if (project_geometry) {
    auto obstacle_input = manifest.value("obstacleInput", Json());
    if (!truth(obstacle_input)) obstacle_input = Json::object();
    const bool random = obstacle_input.value("mode", Json()) == "random";
    if (random && !truth(request.value("run_id", Json()))) throw SceneError("Random geometry requires an explicit run identity");
    if (!frozen || random) working_file = (output / "scene-document.yaml").string();
    if (encoded.size() >= limit) throw SceneError("Prepared world exhausts the output grant");
    const auto remaining = limit - encoded.size();
    Json document;
    std::string resolved;
    if (random && !fs::exists(working_file)) {
      Stage stage(output);
      const auto result = resolve(scene_file, stage.path / "scene-document.yaml", physics.value("generation", Json::object()), remaining);
      document = result.first.first;
      for (const auto& path : fs::directory_iterator(stage.path)) files.emplace_back(output / path.path().filename(), read(path.path(), remaining));
      resolved = working_file;
    } else {
      const auto result = resolve(scene_file, working_file, physics.value("generation", Json::object()), remaining);
      document = result.first.first; resolved = result.second;
    }
    if (document.at("obstacles").size() > 256) throw SceneError("This Gazebo world supports at most 256 scene entities");
    if (!frozen && resolved == scene_file) {
      files.emplace_back(working_file, dumpYaml(document)); scene_file = working_file;
    } else scene_file = resolved;
  }
  std::size_t total = 0;
  Json writes = Json::array();
  for (const auto& file : files) { total += file.second.size(); writes.push_back(file.first.string()); }
  if (total > limit) throw SceneError("Prepared artifacts exceed the output grant");
  Json receipt{{"schema", kPrepareSchema}, {"configuration_revision", revision}, {"worldFile", (output / "prepared-world.world").string()},
               {"sourceWorldSha256", digest(raw)}, {"preparedWorldSha256", digest(encoded)}, {"assetDirectory", asset_directory},
               {"sceneFile", scene_file}, {"workingFile", working_file}, {"frozen", frozen}, {"projectGeometry", project_geometry},
               {"mediaFile", media_file}, {"cameraInfoFile", camera_file}, {"extrinsicFile", extrinsic_file}, {"parameters", physics},
               {"rosData", {{"clockRateHz", physics.at("clock_rate_hz")}}}, {"writes", writes}};
  for (const auto& file : files) atomic(file.first, file.second);
  return receipt;
}

Json inputs(const Json& context, const fs::path& resource_root, const fs::path& workspace_root) {
  const auto& selection = fields(context.at("scene"), {"asset", "simulator", "parameters"}, {"asset", "simulator"});
  const auto& name = selection.at("asset");
  const auto& simulator = selection.at("simulator");
  static const std::regex asset("[A-Za-z0-9][A-Za-z0-9 _-]{0,79}");
  if (!name.is_string() || !std::regex_match(name.get_ref<const std::string&>(), asset) || !trimmed(name.get<std::string>()) ||
      name.get_ref<const std::string&>().find("..") != std::string::npos) throw SceneError("Invalid scene asset name");
  if (!resource_root.is_absolute() || !fs::is_directory(resource_root)) throw SceneError("An explicit existing resource root is required");
  const auto root = fs::canonical(resource_root);
  auto directory = checkedPath((root / "Shared" / "Scenes" / name.get<std::string>()).string(), root / "Shared" / "Scenes", true);
  auto manifest = manifestAt(directory);
  expectRevision(manifest, context.value("expectedRevisionId", Json()));
  const auto supports = manifest.value("simulators", simulator == "none" ? Json::object() : Json());
  const bool physical = simulator == "none";
  if (!supports.is_object() || (!physical && (simulator != "gazebo" && simulator != "xsim")) ||
      (!physical && !supports.value(simulator.get<std::string>(), Json()).is_object()))
    throw SceneError("The selected simulator is not supported by the asset");
  for (const auto& support : supports)
    if (!support.is_object() || support.value("geometry", Json()) != "document")
      throw SceneError("Scene asset geometry must be document");
  auto scene = attachment(directory, manifest.value("sceneDocument", Json()), true);
  auto world = simulator == "gazebo" ? attachment(directory, manifest.value("gazeboWorld", Json())) : "";
  const bool template_world = simulator == "gazebo" && world.empty();
  if (template_world) {
    world = kDefaultWorld;
    if (!fs::is_regular_file(world)) throw SceneError("The provider editable-world template is not installed");
  }
  auto authored = selection.value("parameters", Json::object());
  if (!authored.is_object()) throw SceneError("Scene parameters must be an object");
  authored.erase("lidar");
  if (authored.contains("chassis_robot_ids"))
    throw SceneError("scene parameter chassis_robot_ids is no longer accepted; the Gazebo chassis HOLD roster is dynamic");
  bool locked = false;
  if (authored.contains("frozen")) {
    if (!authored.at("frozen").is_boolean()) throw SceneError("Scene parameter frozen must be a boolean");
    locked = authored.at("frozen").get<bool>();
    authored.erase("frozen");
  }
  const auto physics = simulator == "gazebo" ? parameters(authored, template_world) : authored;
  auto obstacle_input = manifest.value("obstacleInput", Json());
  if (!truth(obstacle_input)) obstacle_input = Json::object();
  const bool random = obstacle_input.value("mode", Json()) == "random";
  const bool editable = !locked;
  std::string working;
  const auto experiment = context.value("experimentResourceId", Json(""));
  if ((editable || random) && truth(experiment)) {
    if (!shortIdentity(experiment)) throw SceneError("A canonical experiment resource identity is required");
    const auto grant = providerPath(workspace_root);
    if (!grant.is_absolute() || !fs::is_directory(grant) || fs::is_symlink(grant))
      throw SceneError("An existing provider workspace grant is required");
    auto output = fs::canonical(grant) / "native-scenes" / experiment.get<std::string>() / name.get<std::string>();
    if (random) {
      const auto run = context.value("sessionId", Json(""));
      if (!shortIdentity(run)) throw SceneError("Random geometry requires the actual Session identity");
      output /= run.get<std::string>();
    }
    workspaceChildren(grant, output);
    working = (output / (random ? fs::path(scene).filename() : fs::path("scene-document.yaml"))).string();
  } else if (random) throw SceneError("Random geometry requires an Experiment workspace");
  if (random) {
    std::vector<fs::path> files;
    for (const auto& entry : fs::recursive_directory_iterator(directory)) {
      const auto status = entry.symlink_status();
      if (fs::is_symlink(status)) throw SceneError("Runtime scene assets cannot include symbolic links");
      if (fs::is_directory(status)) continue;
      if (!fs::is_regular_file(status)) throw SceneError("Runtime scene assets support only regular files");
      if (entry.path().filename() != "manifest.json" && entry.path() != fs::path(scene)) files.push_back(entry.path());
    }
    // Validate sealed references before producing the runtime asset. The
    // expanded document is the only attachment whose bytes and digest change.
    for (const auto* key : {"gazeboWorld", "media", "cameraInfo", "extrinsic", "pointCloud", "preview", "fieldSite", "recordingWindow"})
      attachment(directory, manifest.value(key, Json()));
    if (fs::is_symlink(working)) throw SceneError("Runtime scene document cannot be a symlink");
    const auto expanded = resolve(scene, working, authored.value("generation", Json::object()), 8 * 1024 * 1024);
    if (expanded.second != working) atomic(working, dumpYaml(expanded.first.first));
    scene = working;
    const auto output = fs::path(working).parent_path();
    for (const auto& file : files) {
      const auto target = output / fs::relative(file, directory);
      if (fs::is_symlink(target) || (fs::exists(target) &&
          (!fs::is_regular_file(target) || sha256(target) != sha256(file))))
        throw SceneError("Runtime scene attachment conflicts with an existing file: " + file.filename().string());
      if (!fs::exists(target)) {
        fs::create_directories(target.parent_path());
        fs::copy_file(file, target);
      }
    }
    manifest["sceneDocument"]["sha256"] = sha256(scene);
    manifest["obstacleInput"] = Json{{"mode", "fixed"}, {"format", "geometry"}};
    const auto encoded = dumpJson(manifest);
    const auto target = output / "manifest.json";
    if (!fs::exists(target) || read(target, 65536) != encoded) atomic(target, encoded);
    directory = output;
    if (simulator == "gazebo" && truth(manifest.value("gazeboWorld", Json())))
      world = attachment(directory, manifest.at("gazeboWorld"), true);
  }
  Json value{{"assetDirectory", directory.string()}, {"simulator", simulator}, {"sceneFile", scene}, {"worldFile", world},
             {"sceneDocumentFile", fs::path(scene).filename().string()},
             {"worldAttachment", truth(manifest.value("gazeboWorld", Json())) && simulator == "gazebo"},
             {"projectGeometry", simulator == "gazebo"}, {"workingFile", working}, {"frozen", locked || working.empty()},
             {"parameters", physics}, {"generationParametersJson", ""}};
  // dumpJson supplies Python's number spelling and nonfinite rejection; compact
  // separators are selected without altering spaces inside strings.
  const auto generation_json = authored.value("generation", Json::object());
  std::function<std::string(const Json&)> compact = [&](const Json& item) -> std::string {
    if (item.is_string()) return item.dump(-1, ' ', true);
    if (!item.is_structured()) return dumpJson(item);
    std::string encoded = item.is_object() ? "{" : "[";
    bool first = true;
    for (auto it = item.begin(); it != item.end(); ++it) {
      if (!first) encoded += ',';
      first = false;
      if (item.is_object()) encoded += Json(it.key()).dump(-1, ' ', true) + ":";
      encoded += compact(it.value());
    }
    return encoded + (item.is_object() ? "}" : "]");
  };
  value["generationParametersJson"] = compact(generation_json);
  {
    const auto session = context.value("sessionId", Json());
    if (!shortIdentity(session)) throw SceneError("Compiled scene preparation requires the actual Session identity");
    const auto grant = providerPath(workspace_root);
    if (!grant.is_absolute() || !fs::is_directory(grant) || fs::is_symlink(grant))
      throw SceneError("An existing provider workspace grant is required");
    const auto output = fs::canonical(grant) / "compiled-scenes" / session.get<std::string>();
    workspaceChildren(grant, output);
    // Resolve the editor's selected working definition before compiling. The
    // sealed asset directory remains the resource root even when the working
    // YAML lives elsewhere. Only this finite startup command parses resources.
    const auto initial = !working.empty() && fs::is_regular_file(working)
        ? load(working).first : resolve(scene, working, generation_json).first.first;
    const auto path = output / "scene-document.expanded.json";
    atomic(path, dumpJson(compile(initial, directory.string())));
    value["compiledSceneFile"] = path.string();
  }
  value["mediaFile"] = attachment(directory, manifest.value("media", Json()));
  value["cameraInfoFile"] = attachment(directory, manifest.value("cameraInfo", Json()));
  value["extrinsicFile"] = attachment(directory, manifest.value("extrinsic", Json()));
  // Published consumer contract: workflows bind `scene-inputs` facts.pointCloud.* (e.g. a point-cloud convex
  // cover). The frozen sceneAsset.pointCloud feeds XSim and the adapter runtime; this keeps the published value.
  if (manifest.contains("pointCloud")) value["pointCloud"] = manifest.at("pointCloud");
  return value;
}

Json validate(const fs::path& directory) {
  const auto source = directory / "scene-document.yaml";
  if (fs::is_symlink(source) || !fs::is_regular_file(source)) throw SceneError("scene-document.yaml is required");
  const auto raw = read(source, kMaxDocumentBytes);
  const auto value = parseYaml(raw);
  if (value.is_object() && value.value("schema", Json()) == "xgc2.scene-source.v1" && value.value("mode", Json()) == "random") {
    // A random source has no geometry until its generator runs for a Session.
    // Check the declaration that resolve() checks before it would run it.
    if (value.value("format", Json()) != "geometry") throw SceneError("Scene source requires explicit geometry format and fixed/random mode");
    const auto& generator = value.value("generator", Json());
    const auto command = generator.is_object() ? generator.value("command", Json()) : Json();
    if (!command.is_array() || command.empty() || std::any_of(command.begin(), command.end(), [](const Json& argument) {
          return !argument.is_string() || argument.get_ref<const std::string&>().find('\0') != std::string::npos;
        }))
      throw SceneError("Scene generator requires an explicit argument vector");
    const auto params = generator.value("parameters", Json::object());
    if (!params.is_object() || std::any_of(params.begin(), params.end(), [](const Json& item) { return !finite(item); }))
      throw SceneError("Generator parameter must be a finite number");
    return Json{{"valid", true}, {"mode", "random"}};
  }
  const auto scene = resolve(source).first.first;
  (void)compile(scene, directory.string());
  return Json{{"valid", true}, {"mode", "fixed"}, {"obstacles", scene.at("obstacles").size()}};
}

}  // namespace xgc_scene
