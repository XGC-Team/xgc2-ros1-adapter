// Validation-only interposition of roscpp's real serialize/enqueue operation.
#include <ros/publisher.h>
#include <dlfcn.h>
#include <chrono>
#include <cstdint>
#include <cstdlib>
extern "C" void xgc2_probe_publish(std::uint64_t);
namespace ros {
void Publisher::publish(const boost::function<SerializedMessage(void)> &serialize, SerializedMessage &message) const {
  using Function = void (*)(const Publisher *, const boost::function<SerializedMessage(void)> &, SerializedMessage &);
  static auto original = reinterpret_cast<Function>(dlsym(RTLD_NEXT,
    "_ZNK3ros9Publisher7publishERKN5boost8functionIFNS_17SerializedMessageEvEEERS3_"));
  if (!original) std::abort();
  const auto started = std::chrono::steady_clock::now();
  original(this, serialize, message);
  xgc2_probe_publish(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now()-started).count());
}
}
