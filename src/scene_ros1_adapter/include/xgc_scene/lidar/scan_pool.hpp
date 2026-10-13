#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <mutex>
#include <thread>
#include <vector>

namespace xgc_scene::lidar {

std::size_t defaultScanThreads(std::size_t sensors, unsigned hardware);

// Fixed bounded parallelism, including the tick thread. Each worker takes one
// index at a time. run() is synchronous and not reentrant; no job allocation or
// lock is needed between jobs. The only normal locks are the batch wakeup/join.
class ScanPool {
public:
  explicit ScanPool(std::size_t threads);
  ~ScanPool();
  ScanPool(const ScanPool &) = delete;
  ScanPool &operator=(const ScanPool &) = delete;

  std::size_t threads() const { return workers_.size() + 1; }
  using Job = void (*)(void *, std::size_t);
  // All jobs finish even if one throws; the first exception is rethrown on the
  // caller. context remains borrowed only until run() returns.
  void run(std::size_t count, void *context, Job job, void (*owner_job)(void*) = nullptr);

private:
  void work();
  void drain();
  std::vector<std::thread> workers_;
  std::mutex mutex_;
  std::condition_variable start_, done_;
  Job job_ = nullptr;
  void *context_ = nullptr;
  std::size_t count_ = 0;
  std::atomic<std::size_t> next_{0};
  std::size_t busy_ = 0;
  std::uint64_t batch_ = 0;
  bool stop_ = false;
  std::exception_ptr error_;
};

} // namespace xgc_scene::lidar
