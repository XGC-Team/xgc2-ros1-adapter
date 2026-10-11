#pragma once

#include "xgc_scene/common.hpp"

#include <string>
#include <vector>

namespace xgc_scene::detail {

std::string pythonFloat(double value);
std::string pythonRepr(const Json& value);
std::vector<std::string> utf8Characters(const std::string& value);
unsigned codepoint(const std::string& character);
bool pythonWhitespace(unsigned character);

}  // namespace xgc_scene::detail
