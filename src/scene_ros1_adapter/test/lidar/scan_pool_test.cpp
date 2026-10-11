#include "xgc_scene/lidar/scan_pool.hpp"

#include <gtest/gtest.h>

#include <array>
#include <atomic>
#include <stdexcept>

using namespace xgc_scene::lidar;

TEST(ScanPool, OriginalThreadRule) {
  EXPECT_EQ(1u, defaultScanThreads(0, 8));
  EXPECT_EQ(1u, defaultScanThreads(100, 0));
  EXPECT_EQ(1u, defaultScanThreads(100, 1));
  EXPECT_EQ(4u, defaultScanThreads(100, 8));
  EXPECT_EQ(2u, defaultScanThreads(2, 8));
  EXPECT_EQ(3u, defaultScanThreads(100, 7));
}

TEST(ScanPool, EachIndexExactlyOnceAcrossBatchesAndExceptions) {
  struct Work {
    std::array<std::atomic<unsigned>, 100> calls{};
    bool fail = false;
  } work;
  for (auto threads : {1u, 2u, 4u}) {
    ScanPool pool(threads);
    EXPECT_EQ(threads, pool.threads());
    for (auto &count : work.calls)
      count.store(0);
    for (int batch = 0; batch < 20; ++batch)
      pool.run(work.calls.size(), &work, [](void *context, std::size_t i) {
        auto &w = *static_cast<Work *>(context);
        ++w.calls[i];
        if (w.fail && i == 17)
          throw std::runtime_error("scan refused");
      });
    work.fail = true;
    EXPECT_THROW(pool.run(work.calls.size(), &work,
                          [](void *context, std::size_t i) {
                            auto &w = *static_cast<Work *>(context);
                            ++w.calls[i];
                            if (i == 17)
                              throw std::runtime_error("scan refused");
                          }),
                 std::runtime_error);
    for (const auto &count : work.calls)
      EXPECT_EQ(21u, count.load());
    work.fail = false;
    pool.run(0, nullptr, nullptr);
    EXPECT_THROW(pool.run(1, nullptr,
                          [](void *, std::size_t) {
                            throw std::runtime_error("single scan refused");
                          }),
                 std::runtime_error);
    std::size_t count = 0;
    pool.run(1, &count,
             [](void *p, std::size_t) { ++*static_cast<std::size_t *>(p); });
    EXPECT_EQ(1u, count);
  }
}
