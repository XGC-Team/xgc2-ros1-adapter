#include "xgc_scene/document.hpp"
#include "xgc_scene/json_io.hpp"
#include "xgc_scene/motion.hpp"

#include <gtest/gtest.h>

namespace xgc_scene {
namespace {

TEST(Motion, PythonQuaternionAndPlacementFixtures) {
  // Recorded from motion.py at scene commit 2940a99. Non-unit inputs also
  // verify that composition neither normalizes nor flips quaternion signs.
  const auto a = parseJson("[0.2,-0.3,0.4,0.5]"), b = parseJson("[0.7,0.2,-0.9,0.1]");
  EXPECT_EQ(multiply(a, b), parseJson("[0.56,0.53,-0.16000000000000003,0.33000000000000007]"));
  EXPECT_EQ(rotate(a, parseJson("[1.2,-2.3,3.4]")), parseJson("[0.768,-1.482,1.596]"));
  const Json first{{"position", parseJson("[1.2,-2.3,3.4]")}, {"orientation", a}};
  const Json second{{"position", parseJson("[-4.5,5.6,-6.7]")}, {"orientation", b}};
  EXPECT_EQ(compose(first, second), parseJson(R"({"position":[-0.954,0.17200000000000015,-0.77],"orientation":[0.56,0.53,-0.16000000000000003,0.33000000000000007]})"));
}

TEST(Motion, BackwardsClockAndPausedVelocities) {
  const auto item = obstacle(parseJson(R"({"id":"moving","parts":[{"id":"body","geometry":{"type":"sphere","radius":1}}],"motion":{"type":"ping_pong","point_a":[0,0,0],"point_b":[2,0,0],"speed":1}})"));
  const auto playing = state(item, -1, true), paused = state(item, -1, false);
  EXPECT_EQ(playing.at("pose").at("position"), Json::array({1.0, 0.0, 0.0}));
  EXPECT_EQ(playing.at("linear"), Json::array({-1.0, 0.0, 0.0}));
  EXPECT_EQ(paused.at("pose"), playing.at("pose"));
  EXPECT_EQ(paused.at("linear"), Json::array({0.0, 0.0, 0.0}));
  EXPECT_EQ(paused.at("angular"), Json::array({0.0, 0.0, 0.0}));
}

}  // namespace
}  // namespace xgc_scene
