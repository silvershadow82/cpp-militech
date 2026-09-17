#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

#include "follow/sim/SimTarget.h"

using namespace follow::sim;
using follow::core::Vec3;

TEST(SimTarget, HoldStaysAtStartOnGround)
{
  SimTarget target({3.0, 1.0, -5.0}, {TargetHold{10.0}});
  Vec3 p = target.positionAt(4.0);
  EXPECT_DOUBLE_EQ(p.x, 3.0);
  EXPECT_DOUBLE_EQ(p.y, 1.0);
  EXPECT_DOUBLE_EQ(p.z, 0.0);
}

TEST(SimTarget, LineMovesAtConstantVelocity)
{
  SimTarget target({3.0, 0.0, 0.0}, {TargetLine{.durationS = 10.0, .velNorth = 1.0, .velEast = -0.5}});
  Vec3 p = target.positionAt(2.0);
  EXPECT_NEAR(p.x, 5.0, 1e-12);
  EXPECT_NEAR(p.y, -1.0, 1e-12);
}

TEST(SimTarget, MotionsChainAndStopAfterTheLast)
{
  SimTarget target({0.0, 0.0, 0.0},
                   {TargetHold{1.0}, TargetLine{.durationS = 2.0, .velNorth = 1.0}, TargetLine{.durationS = 1.0, .velEast = 2.0}});

  EXPECT_NEAR(target.positionAt(1.0).x, 0.0, 1e-12);
  EXPECT_NEAR(target.positionAt(2.0).x, 1.0, 1e-12);
  Vec3 end = target.positionAt(100.0);
  EXPECT_NEAR(end.x, 2.0, 1e-12);
  EXPECT_NEAR(end.y, 2.0, 1e-12);
}

TEST(SimTarget, CircleKeepsRadiusAndMovesClockwise)
{
  // Setup: start 6 m south of the center, walking at 1 m/s
  SimTarget target({3.0, 0.0, 0.0}, {TargetCircle{.durationS = 60.0, .centerNed = {9.0, 0.0, 0.0}, .speed = 1.0}});

  // Run: a quarter turn takes 6 * pi / 2 seconds
  Vec3 p = target.positionAt(6.0 * std::numbers::pi / 2.0);

  // Assert: clockwise seen from above goes south -> west
  EXPECT_NEAR(p.x, 9.0, 1e-9);
  EXPECT_NEAR(p.y, -6.0, 1e-9);
}

TEST(SimTarget, OccludedOnlyInsideWindow)
{
  SimTarget target({3.0, 0.0, 0.0}, {TargetHold{10.0}}, {OcclusionWindow{.startS = 2.0, .endS = 3.0}});
  EXPECT_FALSE(target.occludedAt(1.99));
  EXPECT_TRUE(target.occludedAt(2.0));
  EXPECT_TRUE(target.occludedAt(2.99));
  EXPECT_FALSE(target.occludedAt(3.0));
}
