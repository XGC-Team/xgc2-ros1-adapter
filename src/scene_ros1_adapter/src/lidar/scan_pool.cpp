#include "xgc_scene/lidar/scan_pool.hpp"

#include <algorithm>
#include <stdexcept>

namespace xgc_scene::lidar {

std::size_t defaultScanThreads(std::size_t sensors, unsigned hardware) {
  const std::size_t half =
      std::max<std::size_t>(1, (hardware == 0 ? 2 : hardware) / 2);
  return std::max<std::size_t>(1, std::min(sensors, half));
}

ScanPool::ScanPool(std::size_t threads) {
  try {
    workers_.reserve(threads > 0 ? threads - 1 : 0);
    for (std::size_t i = 1; i < threads; ++i)
      workers_.emplace_back([this] { work(); });
  } catch (...) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stop_ = true;
    }
    start_.notify_all();
    for (auto &worker : workers_)
      worker.join();
    throw;
  }
}

ScanPool::~ScanPool() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
  }
  start_.notify_all();
  for (auto &worker : workers_)
    worker.join();
}

void ScanPool::drain() {
  for (;;) {
    const auto i = next_.fetch_add(1, std::memory_order_relaxed);
    if (i >= count_)
      return;
    try {
      job_(context_, i);
    } catch (...) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!error_)
        error_ = std::current_exception();
    }
  }
}

void ScanPool::work() {
  std::uint64_t seen = 0;
  std::unique_lock<std::mutex> lock(mutex_);
  for (;;) {
    start_.wait(lock, [&] { return stop_ || batch_ != seen; });
    if (stop_)
      return;
    seen = batch_;
    lock.unlock();
    drain();
    lock.lock();
    if (--busy_ == 0)
      done_.notify_one();
  }
}

void ScanPool::run(std::size_t count, void *context, Job job) {
  if (count == 0)
    return;
  if (!job)
    throw std::invalid_argument("null scan job");
  if (workers_.empty() || count == 1) {
    std::exception_ptr error;
    for (std::size_t i = 0; i < count; ++i) {
      try {
        job(context, i);
      } catch (...) {
        if (!error)
          error = std::current_exception();
      }
    }
    if (error)
      std::rethrow_exception(error);
    return;
  }
  {
    std::lock_guard<std::mutex> lock(mutex_);
    job_ = job;
    context_ = context;
    count_ = count;
    next_.store(0, std::memory_order_relaxed);
    error_ = nullptr;
    busy_ = workers_.size();
    ++batch_;
  }
  start_.notify_all();
  drain();
  std::exception_ptr error;
  {
    std::unique_lock<std::mutex> lock(mutex_);
    done_.wait(lock, [&] { return busy_ == 0; });
    job_ = nullptr;
    context_ = nullptr;
    std::swap(error, error_);
  }
  if (error)
    std::rethrow_exception(error);
}

} // namespace xgc_scene::lidar
