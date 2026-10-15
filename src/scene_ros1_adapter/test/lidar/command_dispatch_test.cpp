#include "xgc_scene/command_dispatch.hpp"
#include <gtest/gtest.h>
#include <future>
#include <thread>

using namespace std::chrono_literals;
using namespace xgc_scene;

TEST(CommandDispatch, ExpiredQueuedCommandCannotMutateOnLateDispatch) {
  std::function<void()> queued;
  std::atomic<bool> stopping{false};
  int mutations = 0;
  const auto deadline = CommandContext::Clock::now() + 30ms;
  EXPECT_THROW(dispatchCommand([&](auto fn) { queued = std::move(fn); },
      [&] { ++mutations; return Json::object(); }, {deadline, {}}, stopping), CommandExpired);
  ASSERT_TRUE(bool(queued));
  queued();
  EXPECT_EQ(mutations, 0);
}

TEST(CommandDispatch, CancellationBeforeLaneClaimPreventsMutation) {
  std::function<void()> queued;
  std::atomic<bool> stopping{false}, cancelled{false};
  int mutations = 0;
  EXPECT_THROW(dispatchCommand([&](auto fn) { queued = std::move(fn); cancelled = true; },
      [&] { ++mutations; return Json::object(); },
      {CommandContext::Clock::now() + 50ms, [&] { return cancelled.load(); }}, stopping), CommandExpired);
  queued();
  EXPECT_EQ(mutations, 0);
}

TEST(CommandDispatch, ClaimedCommandReturnsItsResult) {
  std::atomic<bool> stopping{false};
  EXPECT_EQ(dispatchCommand([](auto fn) { fn(); }, [] { return Json{{"revision", 7}}; },
      {CommandContext::Clock::now() + 1s, {}}, stopping), (Json{{"revision", 7}}));
}
