#include "scalars.hpp"

#include <cmath>
#include <iomanip>
#include <locale>
#include <sstream>

namespace xgc_scene::detail {

std::string pythonFloat(double value) {
  if (std::isnan(value)) return "nan";
  if (std::isinf(value)) return value < 0 ? "-inf" : "inf";
  if (value == 0) return std::signbit(value) ? "-0.0" : "0.0";

  // Choose the nearest shortest decimal that round-trips, then use Python's
  // fixed/scientific thresholds. GCC 9 has no floating-point std::to_chars.
  std::string scientific;
  for (int digits = 1; digits <= 17; ++digits) {
    std::ostringstream out;
    out.imbue(std::locale::classic());
    out << std::scientific << std::setprecision(digits - 1) << value;
    scientific = out.str();
    std::istringstream in(scientific);
    in.imbue(std::locale::classic());
    double recovered = 0;
    in >> recovered;
    if (!in.fail() && recovered == value) break;
  }
  const auto e = scientific.find('e');
  const int exponent = std::stoi(scientific.substr(e + 1));
  std::string mantissa = scientific.substr(0, e);
  const bool negative = mantissa[0] == '-';
  std::string digits = mantissa.substr(negative ? 1 : 0);
  const auto dot = digits.find('.');
  if (dot != std::string::npos) digits.erase(dot, 1);
  while (digits.size() > 1 && digits.back() == '0') digits.pop_back();
  const std::string sign = negative ? "-" : "";
  if (exponent < -4 || exponent >= 16) {
    std::string result = sign + digits.substr(0, 1);
    if (digits.size() > 1) result += "." + digits.substr(1);
    std::string power = std::to_string(std::abs(exponent));
    if (power.size() < 2) power.insert(0, 1, '0');
    return result + "e" + (exponent < 0 ? "-" : "+") + power;
  }
  const int point = exponent + 1;
  if (point <= 0) return sign + "0." + std::string(-point, '0') + digits;
  if (point >= static_cast<int>(digits.size()))
    return sign + digits + std::string(point - digits.size(), '0') + ".0";
  digits.insert(point, 1, '.');
  return sign + digits;
}

std::vector<std::string> utf8Characters(const std::string& value) {
  std::vector<std::string> result;
  for (std::size_t i = 0; i < value.size();) {
    const auto byte = static_cast<unsigned char>(value[i]);
    const std::size_t count = byte < 0x80 ? 1 : byte < 0xe0 ? 2 : byte < 0xf0 ? 3 : 4;
    if (i + count > value.size()) throw SceneError("Invalid UTF-8 string");
    result.push_back(value.substr(i, count));
    i += count;
  }
  return result;
}

unsigned codepoint(const std::string& character) {
  unsigned value = static_cast<unsigned char>(character[0]);
  if (character.size() == 1) return value;
  value &= character.size() == 2 ? 0x1f : character.size() == 3 ? 0x0f : 0x07;
  for (std::size_t i = 1; i < character.size(); ++i)
    value = (value << 6) | (static_cast<unsigned char>(character[i]) & 0x3f);
  return value;
}

bool pythonWhitespace(unsigned c) {
  return (c >= 9 && c <= 13) || (c >= 0x1c && c <= 0x20) || c == 0x85 || c == 0xa0 || c == 0x1680 ||
         (c >= 0x2000 && c <= 0x200a) || c == 0x2028 || c == 0x2029 || c == 0x202f || c == 0x205f || c == 0x3000;
}

std::string pythonRepr(const Json& value) {
  if (value.is_null()) return "None";
  if (value.is_boolean()) return value.get<bool>() ? "True" : "False";
  if (value.is_number_float()) return pythonFloat(value.get<double>());
  if (value.is_number()) return value.dump();
  if (value.is_string()) {
    const auto& text = value.get_ref<const std::string&>();
    const char quote = text.find('\'') != std::string::npos && text.find('"') == std::string::npos ? '"' : '\'';
    std::string result(1, quote);
    for (const auto& character : utf8Characters(text)) {
      const auto c = codepoint(character);
      if (c == static_cast<unsigned>(quote) || c == '\\') result += "\\" + character;
      else if (c == '\n') result += "\\n";
      else if (c == '\r') result += "\\r";
      else if (c == '\t') result += "\\t";
      else if (c < 0x20 || c == 0x7f || (pythonWhitespace(c) && c != ' ')) {
        std::ostringstream escape;
        escape << (c <= 0xff ? "\\x" : "\\u") << std::hex << std::setfill('0')
               << std::setw(c <= 0xff ? 2 : 4) << c;
        result += escape.str();
      } else result += character;
    }
    return result + quote;
  }
  std::string result = value.is_object() ? "{" : "[";
  bool first = true;
  for (auto it = value.begin(); it != value.end(); ++it) {
    if (!first) result += ", ";
    first = false;
    if (value.is_object()) result += pythonRepr(it.key()) + ": ";
    result += pythonRepr(it.value());
  }
  return result + (value.is_object() ? "}" : "]");
}

}  // namespace xgc_scene::detail
