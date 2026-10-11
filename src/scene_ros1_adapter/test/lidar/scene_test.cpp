#include "xgc_scene/lidar/scene.hpp"

#include <gtest/gtest.h>

using namespace xgc_scene::lidar;
using namespace xgc2_math::scene;

TEST(SceneCache, SharedPoliciesIncrementalRevisionsAndRemoval) {
  SceneCache cache({{0.2, false}, {0.2, false}, {0.1, true}});
  EXPECT_EQ(cache.policyIndex({0.2, false}), 0u);
  EXPECT_EQ(cache.policyIndex({0.1, true}), 1u);
  std::vector<Obstacle> obstacles{Obstacle::box({4, 0, 1}, {1, 4, 2}),
                                  Obstacle::sphere({4, 0, 1}, 1)};
  EXPECT_TRUE(cache.setScene(obstacles, 1));
  const auto first = cache.scene(0);
  EXPECT_FALSE(cache.setScene({}, 1));
  EXPECT_FALSE(cache.setScene({}, 0));
  EXPECT_EQ(first, cache.scene(0));
  obstacles[0].position.x() += 1;
  EXPECT_TRUE(cache.setScene(obstacles, 2));
  EXPECT_NE(first, cache.scene(0));
  for (const auto p : {SamplingPolicy{0.2, false}, SamplingPolicy{0.1, true}}) {
    const LidarScene full(obstacles, p.spacing, p.keep_buried);
    const auto &incremental = *cache.scene(cache.policyIndex(p));
    ASSERT_EQ(full.samples().size(), incremental.samples().size());
    for (std::size_t i = 0; i < full.samples().size(); ++i)
      EXPECT_EQ(full.samples()[i], incremental.samples()[i]);
  }
  EXPECT_TRUE(cache.setScene({}, 3));
  EXPECT_EQ(0u, cache.scene(0)->sampleCount());
  EXPECT_EQ(0u, cache.scene(1)->obstacleCount());
}

TEST(SceneCache, WaitingAndRefusedRevisionsFailClosed) {
  SceneCache cache({{0.1, false}});
  cache.setScene({Obstacle::sphere({3, 0, 0}, 1)}, 1);
  auto previous = cache.scene(0);
  EXPECT_TRUE(cache.invalidate(2));
  EXPECT_FALSE(cache.ready());
  EXPECT_FALSE(cache.invalidate(1));
  EXPECT_THROW(cache.setScene({Obstacle::sphere({3, 0, 0}, -1)}, 2),
               std::invalid_argument);
  EXPECT_FALSE(cache.ready());
  EXPECT_EQ(previous, cache.scene(0));
  EXPECT_TRUE(cache.setScene({}, 2));
  EXPECT_TRUE(cache.ready());
}
