#pragma once

#include "xgc_scene/common.hpp"

#include <string>
#include <utility>
#include <vector>

namespace xgc_scene {

Json uniqueObject(const std::vector<std::pair<Json, Json>>& pairs);
// Like Python json.loads with unique_object, including its nonfinite numeric
// inputs. Validation rejects them; dumpJson also rejects nonfinite numbers.
Json parseJson(const std::string& bytes);
// Python json.dumps(ensure_ascii=False, allow_nan=False), including separators.
std::string dumpJson(const Json& value);

}  // namespace xgc_scene
