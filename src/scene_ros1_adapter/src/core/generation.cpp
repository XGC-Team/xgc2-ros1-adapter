#include "xgc_scene/generation.hpp"

#include "xgc_scene/document.hpp"
#include "xgc_scene/json_io.hpp"
#include "xgc_scene/yaml_io.hpp"
#include "scalars.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <poll.h>
#include <spawn.h>
#include <sys/prctl.h>
#include <sys/wait.h>
#include <system_error>
#include <unistd.h>

extern char** environ;

namespace xgc_scene {
namespace {

namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;

void check(int code) {
  if (code) throw std::system_error(code, std::generic_category());
}

struct Pipe {
  int ends[2] = {-1, -1};
  Pipe() { if (::pipe2(ends, O_CLOEXEC) < 0) check(errno); }
  Pipe(const Pipe&) = delete;
  Pipe& operator=(const Pipe&) = delete;
  void close(std::size_t end) {
    if (ends[end] >= 0) { ::close(ends[end]); ends[end] = -1; }
  }
  ~Pipe() { close(0); close(1); }
};

struct SpawnSetup {
  posix_spawn_file_actions_t actions;
  posix_spawnattr_t attributes;
  SpawnSetup() {
    check(posix_spawn_file_actions_init(&actions));
    const int code = posix_spawnattr_init(&attributes);
    if (code) { posix_spawn_file_actions_destroy(&actions); check(code); }
  }
  ~SpawnSetup() {
    posix_spawnattr_destroy(&attributes);
    posix_spawn_file_actions_destroy(&actions);
  }
};

struct GeneratorGroup {
  static std::atomic_flag subreaper_lock;
  static unsigned subreaper_users;
  static int previous_subreaper;
  pid_t pid = -1;
  GeneratorGroup() {
    while (subreaper_lock.test_and_set(std::memory_order_acquire)) {}
    try {
      if (!subreaper_users) {
        if (::prctl(PR_GET_CHILD_SUBREAPER, &previous_subreaper) < 0) check(errno);
        if (!previous_subreaper && ::prctl(PR_SET_CHILD_SUBREAPER, 1) < 0) check(errno);
      }
      ++subreaper_users;
    } catch (...) { subreaper_lock.clear(std::memory_order_release); throw; }
    subreaper_lock.clear(std::memory_order_release);
  }
  ~GeneratorGroup() {
    if (pid > 0) {
      while (::kill(-pid, SIGKILL) < 0 && errno == EINTR) {}
      // Subreaping lets us wait for orphaned background descendants as well
      // as the leader. Wait only for this owned group, never unrelated children.
      int status;
      while (::waitpid(-pid, &status, 0) > 0 || errno == EINTR) {}
    }
    while (subreaper_lock.test_and_set(std::memory_order_acquire)) {}
    if (!--subreaper_users && !previous_subreaper) ::prctl(PR_SET_CHILD_SUBREAPER, 0);
    subreaper_lock.clear(std::memory_order_release);
  }
};
std::atomic_flag GeneratorGroup::subreaper_lock = ATOMIC_FLAG_INIT;
unsigned GeneratorGroup::subreaper_users = 0;
int GeneratorGroup::previous_subreaper = 0;

int remainingMilliseconds(const Clock::time_point& deadline) {
  const auto remaining = deadline - Clock::now();
  if (remaining <= Clock::duration::zero())
    throw SceneError("Scene generator exceeded its 30 second budget");
  return static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(remaining).count() + 1);
}

std::string readSource(const fs::path& path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) throw std::system_error(errno, std::generic_category(), path.string());
  std::string raw(kMaxDocumentBytes + 1, '\0');
  stream.read(raw.data(), raw.size());
  if (stream.bad()) throw std::system_error(errno, std::generic_category(), path.string());
  raw.resize(static_cast<std::size_t>(stream.gcount()));
  if (raw.size() > kMaxDocumentBytes) throw SceneError("Scene source exceeds the supported size");
  return raw;
}

bool truth(const Json& value) {
  if (value.is_null()) return false;
  if (value.is_boolean()) return value.get<bool>();
  if (value.is_number()) return value.get<double>() != 0;
  return !value.empty() && (!value.is_string() || !value.get_ref<const std::string&>().empty());
}

bool alphanumeric(unsigned c) {
  if (c < 128) return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9');
  // Python 3.8 str.isalnum(), Unicode 12.1. Locale classifiers also accept
  // combining marks and omit numeric characters, so cannot implement this rule.
  static constexpr std::pair<unsigned, unsigned> ranges[] = {
    {0xaa, 0xaa}, {0xb2, 0xb3}, {0xb5, 0xb5}, {0xb9, 0xba},
    {0xbc, 0xbe}, {0xc0, 0xd6}, {0xd8, 0xf6}, {0xf8, 0x2c1},
    {0x2c6, 0x2d1}, {0x2e0, 0x2e4}, {0x2ec, 0x2ec}, {0x2ee, 0x2ee},
    {0x370, 0x374}, {0x376, 0x377}, {0x37a, 0x37d}, {0x37f, 0x37f},
    {0x386, 0x386}, {0x388, 0x38a}, {0x38c, 0x38c}, {0x38e, 0x3a1},
    {0x3a3, 0x3f5}, {0x3f7, 0x481}, {0x48a, 0x52f}, {0x531, 0x556},
    {0x559, 0x559}, {0x560, 0x588}, {0x5d0, 0x5ea}, {0x5ef, 0x5f2},
    {0x620, 0x64a}, {0x660, 0x669}, {0x66e, 0x66f}, {0x671, 0x6d3},
    {0x6d5, 0x6d5}, {0x6e5, 0x6e6}, {0x6ee, 0x6fc}, {0x6ff, 0x6ff},
    {0x710, 0x710}, {0x712, 0x72f}, {0x74d, 0x7a5}, {0x7b1, 0x7b1},
    {0x7c0, 0x7ea}, {0x7f4, 0x7f5}, {0x7fa, 0x7fa}, {0x800, 0x815},
    {0x81a, 0x81a}, {0x824, 0x824}, {0x828, 0x828}, {0x840, 0x858},
    {0x860, 0x86a}, {0x8a0, 0x8b4}, {0x8b6, 0x8bd}, {0x904, 0x939},
    {0x93d, 0x93d}, {0x950, 0x950}, {0x958, 0x961}, {0x966, 0x96f},
    {0x971, 0x980}, {0x985, 0x98c}, {0x98f, 0x990}, {0x993, 0x9a8},
    {0x9aa, 0x9b0}, {0x9b2, 0x9b2}, {0x9b6, 0x9b9}, {0x9bd, 0x9bd},
    {0x9ce, 0x9ce}, {0x9dc, 0x9dd}, {0x9df, 0x9e1}, {0x9e6, 0x9f1},
    {0x9f4, 0x9f9}, {0x9fc, 0x9fc}, {0xa05, 0xa0a}, {0xa0f, 0xa10},
    {0xa13, 0xa28}, {0xa2a, 0xa30}, {0xa32, 0xa33}, {0xa35, 0xa36},
    {0xa38, 0xa39}, {0xa59, 0xa5c}, {0xa5e, 0xa5e}, {0xa66, 0xa6f},
    {0xa72, 0xa74}, {0xa85, 0xa8d}, {0xa8f, 0xa91}, {0xa93, 0xaa8},
    {0xaaa, 0xab0}, {0xab2, 0xab3}, {0xab5, 0xab9}, {0xabd, 0xabd},
    {0xad0, 0xad0}, {0xae0, 0xae1}, {0xae6, 0xaef}, {0xaf9, 0xaf9},
    {0xb05, 0xb0c}, {0xb0f, 0xb10}, {0xb13, 0xb28}, {0xb2a, 0xb30},
    {0xb32, 0xb33}, {0xb35, 0xb39}, {0xb3d, 0xb3d}, {0xb5c, 0xb5d},
    {0xb5f, 0xb61}, {0xb66, 0xb6f}, {0xb71, 0xb77}, {0xb83, 0xb83},
    {0xb85, 0xb8a}, {0xb8e, 0xb90}, {0xb92, 0xb95}, {0xb99, 0xb9a},
    {0xb9c, 0xb9c}, {0xb9e, 0xb9f}, {0xba3, 0xba4}, {0xba8, 0xbaa},
    {0xbae, 0xbb9}, {0xbd0, 0xbd0}, {0xbe6, 0xbf2}, {0xc05, 0xc0c},
    {0xc0e, 0xc10}, {0xc12, 0xc28}, {0xc2a, 0xc39}, {0xc3d, 0xc3d},
    {0xc58, 0xc5a}, {0xc60, 0xc61}, {0xc66, 0xc6f}, {0xc78, 0xc7e},
    {0xc80, 0xc80}, {0xc85, 0xc8c}, {0xc8e, 0xc90}, {0xc92, 0xca8},
    {0xcaa, 0xcb3}, {0xcb5, 0xcb9}, {0xcbd, 0xcbd}, {0xcde, 0xcde},
    {0xce0, 0xce1}, {0xce6, 0xcef}, {0xcf1, 0xcf2}, {0xd05, 0xd0c},
    {0xd0e, 0xd10}, {0xd12, 0xd3a}, {0xd3d, 0xd3d}, {0xd4e, 0xd4e},
    {0xd54, 0xd56}, {0xd58, 0xd61}, {0xd66, 0xd78}, {0xd7a, 0xd7f},
    {0xd85, 0xd96}, {0xd9a, 0xdb1}, {0xdb3, 0xdbb}, {0xdbd, 0xdbd},
    {0xdc0, 0xdc6}, {0xde6, 0xdef}, {0xe01, 0xe30}, {0xe32, 0xe33},
    {0xe40, 0xe46}, {0xe50, 0xe59}, {0xe81, 0xe82}, {0xe84, 0xe84},
    {0xe86, 0xe8a}, {0xe8c, 0xea3}, {0xea5, 0xea5}, {0xea7, 0xeb0},
    {0xeb2, 0xeb3}, {0xebd, 0xebd}, {0xec0, 0xec4}, {0xec6, 0xec6},
    {0xed0, 0xed9}, {0xedc, 0xedf}, {0xf00, 0xf00}, {0xf20, 0xf33},
    {0xf40, 0xf47}, {0xf49, 0xf6c}, {0xf88, 0xf8c}, {0x1000, 0x102a},
    {0x103f, 0x1049}, {0x1050, 0x1055}, {0x105a, 0x105d}, {0x1061, 0x1061},
    {0x1065, 0x1066}, {0x106e, 0x1070}, {0x1075, 0x1081}, {0x108e, 0x108e},
    {0x1090, 0x1099}, {0x10a0, 0x10c5}, {0x10c7, 0x10c7}, {0x10cd, 0x10cd},
    {0x10d0, 0x10fa}, {0x10fc, 0x1248}, {0x124a, 0x124d}, {0x1250, 0x1256},
    {0x1258, 0x1258}, {0x125a, 0x125d}, {0x1260, 0x1288}, {0x128a, 0x128d},
    {0x1290, 0x12b0}, {0x12b2, 0x12b5}, {0x12b8, 0x12be}, {0x12c0, 0x12c0},
    {0x12c2, 0x12c5}, {0x12c8, 0x12d6}, {0x12d8, 0x1310}, {0x1312, 0x1315},
    {0x1318, 0x135a}, {0x1369, 0x137c}, {0x1380, 0x138f}, {0x13a0, 0x13f5},
    {0x13f8, 0x13fd}, {0x1401, 0x166c}, {0x166f, 0x167f}, {0x1681, 0x169a},
    {0x16a0, 0x16ea}, {0x16ee, 0x16f8}, {0x1700, 0x170c}, {0x170e, 0x1711},
    {0x1720, 0x1731}, {0x1740, 0x1751}, {0x1760, 0x176c}, {0x176e, 0x1770},
    {0x1780, 0x17b3}, {0x17d7, 0x17d7}, {0x17dc, 0x17dc}, {0x17e0, 0x17e9},
    {0x17f0, 0x17f9}, {0x1810, 0x1819}, {0x1820, 0x1878}, {0x1880, 0x1884},
    {0x1887, 0x18a8}, {0x18aa, 0x18aa}, {0x18b0, 0x18f5}, {0x1900, 0x191e},
    {0x1946, 0x196d}, {0x1970, 0x1974}, {0x1980, 0x19ab}, {0x19b0, 0x19c9},
    {0x19d0, 0x19da}, {0x1a00, 0x1a16}, {0x1a20, 0x1a54}, {0x1a80, 0x1a89},
    {0x1a90, 0x1a99}, {0x1aa7, 0x1aa7}, {0x1b05, 0x1b33}, {0x1b45, 0x1b4b},
    {0x1b50, 0x1b59}, {0x1b83, 0x1ba0}, {0x1bae, 0x1be5}, {0x1c00, 0x1c23},
    {0x1c40, 0x1c49}, {0x1c4d, 0x1c7d}, {0x1c80, 0x1c88}, {0x1c90, 0x1cba},
    {0x1cbd, 0x1cbf}, {0x1ce9, 0x1cec}, {0x1cee, 0x1cf3}, {0x1cf5, 0x1cf6},
    {0x1cfa, 0x1cfa}, {0x1d00, 0x1dbf}, {0x1e00, 0x1f15}, {0x1f18, 0x1f1d},
    {0x1f20, 0x1f45}, {0x1f48, 0x1f4d}, {0x1f50, 0x1f57}, {0x1f59, 0x1f59},
    {0x1f5b, 0x1f5b}, {0x1f5d, 0x1f5d}, {0x1f5f, 0x1f7d}, {0x1f80, 0x1fb4},
    {0x1fb6, 0x1fbc}, {0x1fbe, 0x1fbe}, {0x1fc2, 0x1fc4}, {0x1fc6, 0x1fcc},
    {0x1fd0, 0x1fd3}, {0x1fd6, 0x1fdb}, {0x1fe0, 0x1fec}, {0x1ff2, 0x1ff4},
    {0x1ff6, 0x1ffc}, {0x2070, 0x2071}, {0x2074, 0x2079}, {0x207f, 0x2089},
    {0x2090, 0x209c}, {0x2102, 0x2102}, {0x2107, 0x2107}, {0x210a, 0x2113},
    {0x2115, 0x2115}, {0x2119, 0x211d}, {0x2124, 0x2124}, {0x2126, 0x2126},
    {0x2128, 0x2128}, {0x212a, 0x212d}, {0x212f, 0x2139}, {0x213c, 0x213f},
    {0x2145, 0x2149}, {0x214e, 0x214e}, {0x2150, 0x2189}, {0x2460, 0x249b},
    {0x24ea, 0x24ff}, {0x2776, 0x2793}, {0x2c00, 0x2c2e}, {0x2c30, 0x2c5e},
    {0x2c60, 0x2ce4}, {0x2ceb, 0x2cee}, {0x2cf2, 0x2cf3}, {0x2cfd, 0x2cfd},
    {0x2d00, 0x2d25}, {0x2d27, 0x2d27}, {0x2d2d, 0x2d2d}, {0x2d30, 0x2d67},
    {0x2d6f, 0x2d6f}, {0x2d80, 0x2d96}, {0x2da0, 0x2da6}, {0x2da8, 0x2dae},
    {0x2db0, 0x2db6}, {0x2db8, 0x2dbe}, {0x2dc0, 0x2dc6}, {0x2dc8, 0x2dce},
    {0x2dd0, 0x2dd6}, {0x2dd8, 0x2dde}, {0x2e2f, 0x2e2f}, {0x3005, 0x3007},
    {0x3021, 0x3029}, {0x3031, 0x3035}, {0x3038, 0x303c}, {0x3041, 0x3096},
    {0x309d, 0x309f}, {0x30a1, 0x30fa}, {0x30fc, 0x30ff}, {0x3105, 0x312f},
    {0x3131, 0x318e}, {0x3192, 0x3195}, {0x31a0, 0x31ba}, {0x31f0, 0x31ff},
    {0x3220, 0x3229}, {0x3248, 0x324f}, {0x3251, 0x325f}, {0x3280, 0x3289},
    {0x32b1, 0x32bf}, {0x3400, 0x4db5}, {0x4e00, 0x9fef}, {0xa000, 0xa48c},
    {0xa4d0, 0xa4fd}, {0xa500, 0xa60c}, {0xa610, 0xa62b}, {0xa640, 0xa66e},
    {0xa67f, 0xa69d}, {0xa6a0, 0xa6ef}, {0xa717, 0xa71f}, {0xa722, 0xa788},
    {0xa78b, 0xa7bf}, {0xa7c2, 0xa7c6}, {0xa7f7, 0xa801}, {0xa803, 0xa805},
    {0xa807, 0xa80a}, {0xa80c, 0xa822}, {0xa830, 0xa835}, {0xa840, 0xa873},
    {0xa882, 0xa8b3}, {0xa8d0, 0xa8d9}, {0xa8f2, 0xa8f7}, {0xa8fb, 0xa8fb},
    {0xa8fd, 0xa8fe}, {0xa900, 0xa925}, {0xa930, 0xa946}, {0xa960, 0xa97c},
    {0xa984, 0xa9b2}, {0xa9cf, 0xa9d9}, {0xa9e0, 0xa9e4}, {0xa9e6, 0xa9fe},
    {0xaa00, 0xaa28}, {0xaa40, 0xaa42}, {0xaa44, 0xaa4b}, {0xaa50, 0xaa59},
    {0xaa60, 0xaa76}, {0xaa7a, 0xaa7a}, {0xaa7e, 0xaaaf}, {0xaab1, 0xaab1},
    {0xaab5, 0xaab6}, {0xaab9, 0xaabd}, {0xaac0, 0xaac0}, {0xaac2, 0xaac2},
    {0xaadb, 0xaadd}, {0xaae0, 0xaaea}, {0xaaf2, 0xaaf4}, {0xab01, 0xab06},
    {0xab09, 0xab0e}, {0xab11, 0xab16}, {0xab20, 0xab26}, {0xab28, 0xab2e},
    {0xab30, 0xab5a}, {0xab5c, 0xab67}, {0xab70, 0xabe2}, {0xabf0, 0xabf9},
    {0xac00, 0xd7a3}, {0xd7b0, 0xd7c6}, {0xd7cb, 0xd7fb}, {0xf900, 0xfa6d},
    {0xfa70, 0xfad9}, {0xfb00, 0xfb06}, {0xfb13, 0xfb17}, {0xfb1d, 0xfb1d},
    {0xfb1f, 0xfb28}, {0xfb2a, 0xfb36}, {0xfb38, 0xfb3c}, {0xfb3e, 0xfb3e},
    {0xfb40, 0xfb41}, {0xfb43, 0xfb44}, {0xfb46, 0xfbb1}, {0xfbd3, 0xfd3d},
    {0xfd50, 0xfd8f}, {0xfd92, 0xfdc7}, {0xfdf0, 0xfdfb}, {0xfe70, 0xfe74},
    {0xfe76, 0xfefc}, {0xff10, 0xff19}, {0xff21, 0xff3a}, {0xff41, 0xff5a},
    {0xff66, 0xffbe}, {0xffc2, 0xffc7}, {0xffca, 0xffcf}, {0xffd2, 0xffd7},
    {0xffda, 0xffdc}, {0x10000, 0x1000b}, {0x1000d, 0x10026}, {0x10028, 0x1003a},
    {0x1003c, 0x1003d}, {0x1003f, 0x1004d}, {0x10050, 0x1005d}, {0x10080, 0x100fa},
    {0x10107, 0x10133}, {0x10140, 0x10178}, {0x1018a, 0x1018b}, {0x10280, 0x1029c},
    {0x102a0, 0x102d0}, {0x102e1, 0x102fb}, {0x10300, 0x10323}, {0x1032d, 0x1034a},
    {0x10350, 0x10375}, {0x10380, 0x1039d}, {0x103a0, 0x103c3}, {0x103c8, 0x103cf},
    {0x103d1, 0x103d5}, {0x10400, 0x1049d}, {0x104a0, 0x104a9}, {0x104b0, 0x104d3},
    {0x104d8, 0x104fb}, {0x10500, 0x10527}, {0x10530, 0x10563}, {0x10600, 0x10736},
    {0x10740, 0x10755}, {0x10760, 0x10767}, {0x10800, 0x10805}, {0x10808, 0x10808},
    {0x1080a, 0x10835}, {0x10837, 0x10838}, {0x1083c, 0x1083c}, {0x1083f, 0x10855},
    {0x10858, 0x10876}, {0x10879, 0x1089e}, {0x108a7, 0x108af}, {0x108e0, 0x108f2},
    {0x108f4, 0x108f5}, {0x108fb, 0x1091b}, {0x10920, 0x10939}, {0x10980, 0x109b7},
    {0x109bc, 0x109cf}, {0x109d2, 0x10a00}, {0x10a10, 0x10a13}, {0x10a15, 0x10a17},
    {0x10a19, 0x10a35}, {0x10a40, 0x10a48}, {0x10a60, 0x10a7e}, {0x10a80, 0x10a9f},
    {0x10ac0, 0x10ac7}, {0x10ac9, 0x10ae4}, {0x10aeb, 0x10aef}, {0x10b00, 0x10b35},
    {0x10b40, 0x10b55}, {0x10b58, 0x10b72}, {0x10b78, 0x10b91}, {0x10ba9, 0x10baf},
    {0x10c00, 0x10c48}, {0x10c80, 0x10cb2}, {0x10cc0, 0x10cf2}, {0x10cfa, 0x10d23},
    {0x10d30, 0x10d39}, {0x10e60, 0x10e7e}, {0x10f00, 0x10f27}, {0x10f30, 0x10f45},
    {0x10f51, 0x10f54}, {0x10fe0, 0x10ff6}, {0x11003, 0x11037}, {0x11052, 0x1106f},
    {0x11083, 0x110af}, {0x110d0, 0x110e8}, {0x110f0, 0x110f9}, {0x11103, 0x11126},
    {0x11136, 0x1113f}, {0x11144, 0x11144}, {0x11150, 0x11172}, {0x11176, 0x11176},
    {0x11183, 0x111b2}, {0x111c1, 0x111c4}, {0x111d0, 0x111da}, {0x111dc, 0x111dc},
    {0x111e1, 0x111f4}, {0x11200, 0x11211}, {0x11213, 0x1122b}, {0x11280, 0x11286},
    {0x11288, 0x11288}, {0x1128a, 0x1128d}, {0x1128f, 0x1129d}, {0x1129f, 0x112a8},
    {0x112b0, 0x112de}, {0x112f0, 0x112f9}, {0x11305, 0x1130c}, {0x1130f, 0x11310},
    {0x11313, 0x11328}, {0x1132a, 0x11330}, {0x11332, 0x11333}, {0x11335, 0x11339},
    {0x1133d, 0x1133d}, {0x11350, 0x11350}, {0x1135d, 0x11361}, {0x11400, 0x11434},
    {0x11447, 0x1144a}, {0x11450, 0x11459}, {0x1145f, 0x1145f}, {0x11480, 0x114af},
    {0x114c4, 0x114c5}, {0x114c7, 0x114c7}, {0x114d0, 0x114d9}, {0x11580, 0x115ae},
    {0x115d8, 0x115db}, {0x11600, 0x1162f}, {0x11644, 0x11644}, {0x11650, 0x11659},
    {0x11680, 0x116aa}, {0x116b8, 0x116b8}, {0x116c0, 0x116c9}, {0x11700, 0x1171a},
    {0x11730, 0x1173b}, {0x11800, 0x1182b}, {0x118a0, 0x118f2}, {0x118ff, 0x118ff},
    {0x119a0, 0x119a7}, {0x119aa, 0x119d0}, {0x119e1, 0x119e1}, {0x119e3, 0x119e3},
    {0x11a00, 0x11a00}, {0x11a0b, 0x11a32}, {0x11a3a, 0x11a3a}, {0x11a50, 0x11a50},
    {0x11a5c, 0x11a89}, {0x11a9d, 0x11a9d}, {0x11ac0, 0x11af8}, {0x11c00, 0x11c08},
    {0x11c0a, 0x11c2e}, {0x11c40, 0x11c40}, {0x11c50, 0x11c6c}, {0x11c72, 0x11c8f},
    {0x11d00, 0x11d06}, {0x11d08, 0x11d09}, {0x11d0b, 0x11d30}, {0x11d46, 0x11d46},
    {0x11d50, 0x11d59}, {0x11d60, 0x11d65}, {0x11d67, 0x11d68}, {0x11d6a, 0x11d89},
    {0x11d98, 0x11d98}, {0x11da0, 0x11da9}, {0x11ee0, 0x11ef2}, {0x11fc0, 0x11fd4},
    {0x12000, 0x12399}, {0x12400, 0x1246e}, {0x12480, 0x12543}, {0x13000, 0x1342e},
    {0x14400, 0x14646}, {0x16800, 0x16a38}, {0x16a40, 0x16a5e}, {0x16a60, 0x16a69},
    {0x16ad0, 0x16aed}, {0x16b00, 0x16b2f}, {0x16b40, 0x16b43}, {0x16b50, 0x16b59},
    {0x16b5b, 0x16b61}, {0x16b63, 0x16b77}, {0x16b7d, 0x16b8f}, {0x16e40, 0x16e96},
    {0x16f00, 0x16f4a}, {0x16f50, 0x16f50}, {0x16f93, 0x16f9f}, {0x16fe0, 0x16fe1},
    {0x16fe3, 0x16fe3}, {0x17000, 0x187f7}, {0x18800, 0x18af2}, {0x1b000, 0x1b11e},
    {0x1b150, 0x1b152}, {0x1b164, 0x1b167}, {0x1b170, 0x1b2fb}, {0x1bc00, 0x1bc6a},
    {0x1bc70, 0x1bc7c}, {0x1bc80, 0x1bc88}, {0x1bc90, 0x1bc99}, {0x1d2e0, 0x1d2f3},
    {0x1d360, 0x1d378}, {0x1d400, 0x1d454}, {0x1d456, 0x1d49c}, {0x1d49e, 0x1d49f},
    {0x1d4a2, 0x1d4a2}, {0x1d4a5, 0x1d4a6}, {0x1d4a9, 0x1d4ac}, {0x1d4ae, 0x1d4b9},
    {0x1d4bb, 0x1d4bb}, {0x1d4bd, 0x1d4c3}, {0x1d4c5, 0x1d505}, {0x1d507, 0x1d50a},
    {0x1d50d, 0x1d514}, {0x1d516, 0x1d51c}, {0x1d51e, 0x1d539}, {0x1d53b, 0x1d53e},
    {0x1d540, 0x1d544}, {0x1d546, 0x1d546}, {0x1d54a, 0x1d550}, {0x1d552, 0x1d6a5},
    {0x1d6a8, 0x1d6c0}, {0x1d6c2, 0x1d6da}, {0x1d6dc, 0x1d6fa}, {0x1d6fc, 0x1d714},
    {0x1d716, 0x1d734}, {0x1d736, 0x1d74e}, {0x1d750, 0x1d76e}, {0x1d770, 0x1d788},
    {0x1d78a, 0x1d7a8}, {0x1d7aa, 0x1d7c2}, {0x1d7c4, 0x1d7cb}, {0x1d7ce, 0x1d7ff},
    {0x1e100, 0x1e12c}, {0x1e137, 0x1e13d}, {0x1e140, 0x1e149}, {0x1e14e, 0x1e14e},
    {0x1e2c0, 0x1e2eb}, {0x1e2f0, 0x1e2f9}, {0x1e800, 0x1e8c4}, {0x1e8c7, 0x1e8cf},
    {0x1e900, 0x1e943}, {0x1e94b, 0x1e94b}, {0x1e950, 0x1e959}, {0x1ec71, 0x1ecab},
    {0x1ecad, 0x1ecaf}, {0x1ecb1, 0x1ecb4}, {0x1ed01, 0x1ed2d}, {0x1ed2f, 0x1ed3d},
    {0x1ee00, 0x1ee03}, {0x1ee05, 0x1ee1f}, {0x1ee21, 0x1ee22}, {0x1ee24, 0x1ee24},
    {0x1ee27, 0x1ee27}, {0x1ee29, 0x1ee32}, {0x1ee34, 0x1ee37}, {0x1ee39, 0x1ee39},
    {0x1ee3b, 0x1ee3b}, {0x1ee42, 0x1ee42}, {0x1ee47, 0x1ee47}, {0x1ee49, 0x1ee49},
    {0x1ee4b, 0x1ee4b}, {0x1ee4d, 0x1ee4f}, {0x1ee51, 0x1ee52}, {0x1ee54, 0x1ee54},
    {0x1ee57, 0x1ee57}, {0x1ee59, 0x1ee59}, {0x1ee5b, 0x1ee5b}, {0x1ee5d, 0x1ee5d},
    {0x1ee5f, 0x1ee5f}, {0x1ee61, 0x1ee62}, {0x1ee64, 0x1ee64}, {0x1ee67, 0x1ee6a},
    {0x1ee6c, 0x1ee72}, {0x1ee74, 0x1ee77}, {0x1ee79, 0x1ee7c}, {0x1ee7e, 0x1ee7e},
    {0x1ee80, 0x1ee89}, {0x1ee8b, 0x1ee9b}, {0x1eea1, 0x1eea3}, {0x1eea5, 0x1eea9},
    {0x1eeab, 0x1eebb}, {0x1f100, 0x1f10c}, {0x20000, 0x2a6d6}, {0x2a700, 0x2b734},
    {0x2b740, 0x2b81d}, {0x2b820, 0x2cea1}, {0x2ceb0, 0x2ebe0}, {0x2f800, 0x2fa1d},
  };
  const auto found = std::lower_bound(std::begin(ranges), std::end(ranges), c,
                                    [](const auto& range, unsigned value) { return range.second < value; });
  return found != std::end(ranges) && found->first <= c;
}

bool parameterName(const std::string& key) {
  if (key.empty() || key[0] == '-') return false;
  for (const auto& character : detail::utf8Characters(key)) {
    const auto c = detail::codepoint(character);
    if (c != '-' && !alphanumeric(c)) return false;
  }
  return true;
}

// Generator output uses json.loads, which keeps the last duplicate field.
// This local decoder preserves its grammar diagnostics and delegates scalar
// decoding to the base JSON parser, including Python's nonfinite literals.
class GeneratorJson {
 public:
  explicit GeneratorJson(const std::string& bytes)
      : bytes_(bytes.compare(0, 3, "\xef\xbb\xbf") == 0 ? bytes.substr(3) : bytes) {}
  Json parse() {
    Json result = value();
    whitespace();
    if (at_ != bytes_.size()) error("Extra data", at_);
    return result;
  }

 private:
  const std::string bytes_;
  std::size_t at_ = 0;

  [[noreturn]] void error(const std::string& message, std::size_t position) const {
    const auto line = 1 + std::count(bytes_.begin(), bytes_.begin() + position, '\n');
    const auto previous = position == 0 ? std::string::npos : bytes_.rfind('\n', position - 1);
    const auto line_start = previous == std::string::npos ? 0 : previous + 1;
    const auto column = detail::utf8Characters(bytes_.substr(line_start, position - line_start)).size() + 1;
    const auto character = detail::utf8Characters(bytes_.substr(0, position)).size();
    throw SceneError("Invalid generator result: " + message + ": line " + std::to_string(line) +
                     " column " + std::to_string(column) + " (char " + std::to_string(character) + ")");
  }
  void whitespace() {
    while (at_ < bytes_.size() && std::string(" \t\r\n").find(bytes_[at_]) != std::string::npos) ++at_;
  }
  bool take(char c) {
    if (at_ < bytes_.size() && bytes_[at_] == c) { ++at_; return true; }
    return false;
  }
  Json string() {
    const auto start = at_++;
    while (at_ < bytes_.size()) {
      const auto c = static_cast<unsigned char>(bytes_[at_++]);
      if (c == '"') return parseJson(bytes_.substr(start, at_ - start));
      if (c < 0x20) error("Invalid control character at", at_ - 1);
      if (c == '\\') {
        if (at_ == bytes_.size()) break;
        const char escaped = bytes_[at_++];
        if (escaped == 'u') {
          const auto u = at_ - 1;
          for (int i = 0; i < 4; ++i) {
            if (at_ == bytes_.size() || std::string("0123456789abcdefABCDEF").find(bytes_[at_]) == std::string::npos)
              error("Invalid \\uXXXX escape", u);
            ++at_;
          }
        } else if (std::string("\"\\/bfnrt").find(escaped) == std::string::npos)
          error("Invalid \\escape", at_ - 2);
      }
    }
    error("Unterminated string starting at", start);
  }
  Json value() {
    whitespace();
    if (at_ == bytes_.size()) error("Expecting value", at_);
    if (bytes_[at_] == '"') return string();
    if (take('{')) {
      Json object = Json::object();
      whitespace();
      if (take('}')) return object;
      for (;;) {
        if (at_ == bytes_.size() || bytes_[at_] != '"') error("Expecting property name enclosed in double quotes", at_);
        const auto key = string().get<std::string>();
        whitespace();
        if (!take(':')) error("Expecting ':' delimiter", at_);
        object[key] = value();
        whitespace();
        if (take('}')) return object;
        if (!take(',')) error("Expecting ',' delimiter", at_);
        whitespace();
      }
    }
    if (take('[')) {
      Json array = Json::array();
      whitespace();
      if (take(']')) return array;
      for (;;) {
        array.push_back(value());
        whitespace();
        if (take(']')) return array;
        if (!take(',')) error("Expecting ',' delimiter", at_);
      }
    }
    for (const auto* literal : {"true", "false", "null", "NaN", "Infinity", "-Infinity"}) {
      const auto length = std::strlen(literal);
      if (bytes_.compare(at_, length, literal) == 0) { at_ += length; return parseJson(literal); }
    }
    const auto start = at_;
    auto end = at_;
    if (end < bytes_.size() && bytes_[end] == '-') ++end;
    if (end < bytes_.size() && bytes_[end] == '0') ++end;
    else if (end < bytes_.size() && bytes_[end] >= '1' && bytes_[end] <= '9')
      while (end < bytes_.size() && bytes_[end] >= '0' && bytes_[end] <= '9') ++end;
    else error("Expecting value", start);
    const auto integer_end = end;
    if (end < bytes_.size() && bytes_[end] == '.') {
      ++end;
      const auto fraction = end;
      while (end < bytes_.size() && bytes_[end] >= '0' && bytes_[end] <= '9') ++end;
      if (end == fraction) end = integer_end;
    }
    const auto fraction_end = end;
    if (end < bytes_.size() && (bytes_[end] == 'e' || bytes_[end] == 'E')) {
      ++end;
      if (end < bytes_.size() && (bytes_[end] == '+' || bytes_[end] == '-')) ++end;
      const auto exponent = end;
      while (end < bytes_.size() && bytes_[end] >= '0' && bytes_[end] <= '9') ++end;
      if (end == exponent) end = fraction_end;
    }
    at_ = end;
    return parseJson(bytes_.substr(start, end - start));
  }
};

std::string pythonType(const Json& value) {
  if (value.is_null()) return "NoneType";
  if (value.is_boolean()) return "bool";
  if (value.is_number_float()) return "float";
  return "int";
}

fs::path metadataPath(const fs::path& target) {
  // pathlib leaves a trailing dot and a leading dot outside the suffix.
  const auto name = target.filename().string();
  const auto dot = name.rfind('.');
  const auto stem = dot != std::string::npos && dot > 0 && dot + 1 < name.size() ? name.substr(0, dot) : name;
  return target.parent_path() / (stem + ".generation.yaml");
}

void writeArtifact(const fs::path& path, const std::string& body) {
  auto temporary = (path.parent_path() / "tmpXXXXXX").string();
  const int fd = ::mkostemp(temporary.data(), O_CLOEXEC);
  if (fd < 0) check(errno);
  try {
    std::size_t written = 0;
    while (written < body.size()) {
      const auto count = ::write(fd, body.data() + written, body.size() - written);
      if (count < 0) { if (errno == EINTR) continue; check(errno); }
      if (count == 0) check(EIO);
      written += static_cast<std::size_t>(count);
    }
    while (::fsync(fd) < 0) { if (errno != EINTR) check(errno); }
    if (::rename(temporary.c_str(), path.c_str()) < 0) check(errno);
  } catch (...) {
    ::close(fd);
    ::unlink(temporary.c_str());
    throw;
  }
  ::close(fd);
}

}  // namespace

std::string run_generator(const std::vector<std::string>& argv, std::size_t output_limit) {
  if (argv.empty()) throw SceneError("Scene generator requires an explicit argument vector");
  std::vector<char*> arguments;
  for (const auto& argument : argv) arguments.push_back(const_cast<char*>(argument.c_str()));
  arguments.push_back(nullptr);
  GeneratorGroup group;
  Pipe output, errors;
  SpawnSetup setup;
  sigset_t defaults;
  ::sigemptyset(&defaults);
  ::sigaddset(&defaults, SIGPIPE);
  ::sigaddset(&defaults, SIGXFSZ);
  check(posix_spawnattr_setsigdefault(&setup.attributes, &defaults));
  check(posix_spawnattr_setflags(&setup.attributes, POSIX_SPAWN_SETSID | POSIX_SPAWN_SETSIGDEF));
  check(posix_spawn_file_actions_adddup2(&setup.actions, output.ends[1], STDOUT_FILENO));
  check(posix_spawn_file_actions_adddup2(&setup.actions, errors.ends[1], STDERR_FILENO));
  for (const int fd : {output.ends[0], output.ends[1], errors.ends[0], errors.ends[1]})
    check(posix_spawn_file_actions_addclose(&setup.actions, fd));
  check(posix_spawnp(&group.pid, arguments[0], &setup.actions, &setup.attributes, arguments.data(), environ));
  output.close(1);
  errors.close(1);
  const auto deadline = Clock::now() + std::chrono::seconds(30);
  std::array<pollfd, 2> streams{{{output.ends[0], POLLIN, 0}, {errors.ends[0], POLLIN, 0}}};
  std::array<std::string, 2> buffers;
  const std::array<std::size_t, 2> limits{{output_limit, 65536}};
  const std::array<const char*, 2> names{{"stdout", "stderr"}};
  std::array<char, 65536> chunk;
  while (streams[0].fd >= 0 || streams[1].fd >= 0) {
    const int ready = ::poll(streams.data(), streams.size(), remainingMilliseconds(deadline));
    if (ready < 0) { if (errno == EINTR) continue; check(errno); }
    for (std::size_t i = 0; i < streams.size(); ++i) {
      if (!streams[i].revents) continue;
      const auto count = ::read(streams[i].fd, chunk.data(), chunk.size());
      if (count < 0) { if (errno == EINTR) continue; check(errno); }
      if (!count) streams[i].fd = -1;
      else if (static_cast<std::size_t>(count) > limits[i] - buffers[i].size())
        throw SceneError(std::string("Scene generator ") + names[i] + " exceeds its byte limit");
      else buffers[i].append(chunk.data(), static_cast<std::size_t>(count));
    }
  }
  int status;
  for (;;) {
    const auto result = ::waitpid(group.pid, &status, WNOHANG);
    if (result == group.pid) break;
    if (result < 0) { if (errno == EINTR) continue; check(errno); }
    const auto remaining = remainingMilliseconds(deadline);
    if (::poll(nullptr, 0, std::min(remaining, 10)) < 0 && errno != EINTR) check(errno);
  }
  const int code = WIFSIGNALED(status) ? -WTERMSIG(status) : WEXITSTATUS(status);
  if (code) throw SceneError("Scene generator exited with status " + std::to_string(code));
  return buffers[0];
}

std::pair<std::pair<Json, std::string>, std::string> resolve(
    const fs::path& source, const fs::path& working_file, const Json& overrides, std::size_t max_bytes) {
  const auto raw = readSource(source);
  const auto value = parseYaml(raw);
  if (!value.is_object() || value.value("schema", Json(nullptr)) != "xgc2.scene-source.v1")
    return {{document(value), digest(raw)}, source.string()};
  const auto mode = value.value("mode", Json(nullptr));
  if (value.value("format", Json(nullptr)) != "geometry" || (mode != "fixed" && mode != "random"))
    throw SceneError("Scene source requires explicit geometry format and fixed/random mode");
  if (mode == "fixed") return {{document(value.at("document")), digest(raw)}, source.string()};
  if (working_file.empty()) throw SceneError("Random scene requires a run-owned result file");
  if (fs::is_regular_file(working_file)) return {load(working_file), working_file.string()};
  const auto& generator = value.at("generator");
  const auto command = generator.value("command", Json(nullptr));
  if (!command.is_array() || command.empty() || std::any_of(command.begin(), command.end(), [](const Json& argument) {
        return !argument.is_string() || argument.get_ref<const std::string&>().find('\0') != std::string::npos;
      }))
    throw SceneError("Scene generator requires an explicit argument vector");
  Json params = generator.value("parameters", Json::object());
  const Json requested = truth(overrides) ? overrides : Json::object();
  if (!params.is_object() || !requested.is_object())
    throw SceneError("Scene generation parameters are not declared by this generator");
  for (auto it = requested.begin(); it != requested.end(); ++it)
    if (!params.contains(it.key())) throw SceneError("Scene generation parameters are not declared by this generator");
  for (auto it = requested.begin(); it != requested.end(); ++it) params[it.key()] = it.value();
  auto argv = command.get<std::vector<std::string>>();
  for (auto it = params.begin(); it != params.end(); ++it) {
    if (!parameterName(it.key())) throw SceneError("Invalid generator parameter name");
    if (!it.value().is_number() || !std::isfinite(it.value().get<double>()))
      throw SceneError("Generator parameter must be a finite number");
    argv.push_back("--" + it.key());
    argv.push_back(it.value().is_number_float() ? detail::pythonFloat(it.value().get<double>()) : it.value().dump());
  }
  std::string stdout_bytes;
  try { stdout_bytes = run_generator(argv, std::min(kMaxDocumentBytes, max_bytes)); }
  catch (const std::system_error& error) {
    throw SceneError("Scene generator failed: [Errno " + std::to_string(error.code().value()) + "] " +
                     error.code().message() + ": " + detail::pythonRepr(command[0]));
  }
  auto produced = GeneratorJson(stdout_bytes).parse();
  if (!produced.is_object()) {
    const auto message = produced.is_array() ? "list indices must be integers or slices, not str" :
                         produced.is_string() ? "string indices must be integers" :
                         "'" + pythonType(produced) + "' object is not subscriptable";
    throw SceneError("Invalid generator result: " + message);
  }
  if (!produced.contains("scene")) throw SceneError("Invalid generator result: 'scene'");
  Json generated;
  try { generated = document(produced.at("scene")); }
  catch (const SceneError& error) { throw SceneError("Invalid generator result: " + std::string(error.what())); }
  fs::create_directories(working_file.parent_path().empty() ? fs::path(".") : working_file.parent_path());
  produced.erase("scene");
  produced["parameters"] = params;
  produced["sourceSha256"] = digest(raw);
  produced["mode"] = "random";
  produced["format"] = "geometry";
  const auto metadata = dumpYaml(produced), scene = dumpYaml(generated);
  if (metadata.size() > max_bytes || scene.size() > max_bytes - metadata.size())
    throw SceneError("Generated scene artifacts exceed the granted byte limit");
  writeArtifact(metadataPath(working_file), metadata);
  writeArtifact(working_file, scene);
  return {load(working_file), working_file.string()};
}

}  // namespace xgc_scene
