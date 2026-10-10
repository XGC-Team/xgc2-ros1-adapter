#include "xgc_ugv_ros1_adapter/variant.hpp"

#include "xgc_ugv_ros1_adapter/mecanum_ugv_contract.hpp"
#include "xgc_ugv_ros1_adapter/scout_mini_contract.hpp"

namespace xgc_ugv_ros1_adapter {

const UgvVariant &scoutMini() {
  static const UgvVariant variant{
      "xgc2-scout-mini-ros1-adapter",
      "Scout Mini",
      "Scout",
      "scout-mini",
      {1.5, 0.5235, false},
      HealthSource::kChassisState,
      true,
      0.0,
      scout_mini::InstalledContract(),
  };
  return variant;
}

const UgvVariant &mecanumUgv() {
  static const UgvVariant variant{
      "xgc2-mecanum-ugv-ros1-adapter",
      "Mecanum UGV",
      "Mecanum",
      "mecanum-ugv",
      // Wheeltec Mecanum chassis capability. These are not Scout SDK limits
      // and not the old swarm-sync-sim 1.5 m/s remote scale.
      {1.0, 1.5707963267948966, true},
      HealthSource::kImu,
      false,
      1.0,
      mecanum_ugv::InstalledContract(),
  };
  return variant;
}

const UgvVariant *variantFor(const std::string &provider) {
  for (const UgvVariant *variant : {&scoutMini(), &mecanumUgv()})
    if (provider == variant->provider)
      return variant;
  return nullptr;
}

} // namespace xgc_ugv_ros1_adapter
