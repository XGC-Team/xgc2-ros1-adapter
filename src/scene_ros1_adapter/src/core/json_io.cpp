#include "xgc_scene/json_io.hpp"

#include "scalars.hpp"

#include <cmath>
#include <cstdlib>
#include <limits>
#include <optional>
#include <regex>
#include <set>

namespace xgc_scene {

Json uniqueObject(const std::vector<std::pair<Json, Json>>& pairs) {
  Json result = Json::object();
  for (const auto& pair : pairs) {
    if (!pair.first.is_string()) throw SceneError("Scene field names must be strings");
    const auto& key = pair.first.get_ref<const std::string&>();
    if (result.contains(key)) throw SceneError("Duplicate scene field " + key);
    result[key] = pair.second;
  }
  return result;
}

Json parseJson(const std::string& bytes) {
  // Preserve Python's nonfinite literals and overflowing numbers while letting
  // nlohmann validate the JSON grammar and decode strings. Each numeric leaf
  // has one callback in input order, including leaves in nested containers.
  std::string input;
  std::vector<std::optional<double>> numbers;
  static const std::regex numeric(R"(-?(0|[1-9][0-9]*)(\.[0-9]+)?([eE][+-]?[0-9]+)?)");
  for (std::size_t i = 0; i < bytes.size();) {
    if (bytes[i] == '"') {
      const auto start = i++;
      while (i < bytes.size()) {
        if (bytes[i] == '\\') { i += std::min<std::size_t>(2, bytes.size() - i); continue; }
        if (bytes[i++] == '"') break;
      }
      input.append(bytes, start, i - start);
      continue;
    }
    if (bytes[i] == '-' || (bytes[i] >= '0' && bytes[i] <= '9') || bytes[i] == 'N' || bytes[i] == 'I') {
      const auto start = i;
      while (i < bytes.size() && std::string(" \t\r\n,]}:{[\"").find(bytes[i]) == std::string::npos) ++i;
      const auto token = bytes.substr(start, i - start);
      std::optional<double> replacement;
      if (token == "NaN") replacement = std::numeric_limits<double>::quiet_NaN();
      else if (token == "Infinity" || token == "-Infinity")
        replacement = token[0] == '-' ? -std::numeric_limits<double>::infinity() : std::numeric_limits<double>::infinity();
      else if (std::regex_match(token, numeric)) {
        const double value = std::strtod(token.c_str(), nullptr);
        if (!std::isfinite(value)) replacement = value;
      } else {
        input += token;
        continue;
      }
      numbers.push_back(replacement);
      input += replacement ? "0.0" : token;
    } else input += bytes[i++];
  }
  std::vector<std::set<std::string>> keys;
  std::size_t number_index = 0;
  return Json::parse(input, [&](int, Json::parse_event_t event, Json& value) {
    if (event == Json::parse_event_t::object_start) keys.emplace_back();
    else if (event == Json::parse_event_t::object_end) keys.pop_back();
    else if (event == Json::parse_event_t::key) {
      const auto key = value.get<std::string>();
      if (!keys.back().insert(key).second) throw SceneError("Duplicate scene field " + key);
    } else if (event == Json::parse_event_t::value && value.is_number()) {
      if (numbers.at(number_index)) value = *numbers[number_index];
      ++number_index;
    }
    return true;
  });
}

std::string dumpJson(const Json& value) {
  if (value.is_number_float()) {
    if (!std::isfinite(value.get<double>())) throw SceneError("Out of range float values are not JSON compliant");
    return detail::pythonFloat(value.get<double>());
  }
  if (!value.is_structured()) return value.dump(-1, ' ', false);
  std::string result = value.is_object() ? "{" : "[";
  bool first = true;
  for (auto it = value.begin(); it != value.end(); ++it) {
    if (!first) result += ", ";
    first = false;
    if (value.is_object()) result += Json(it.key()).dump(-1, ' ', false) + ": ";
    result += dumpJson(it.value());
  }
  return result + (value.is_object() ? "}" : "]");
}

}  // namespace xgc_scene
