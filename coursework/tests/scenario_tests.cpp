#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

#include "follow/core/Angles.h"
#include "follow/sim/ScenarioRunner.h"

using namespace follow;
using namespace follow::sim;
using S = core::State;

namespace {

std::optional<double> followingStart(const ScenarioResult& result)
{
  for (const StepRecord& step : result.steps) {
    if (step.state == S::Following) {
      return step.tS;
    }
  }
  return std::nullopt;
}

double minTrueDistance(const ScenarioResult& result)
{
  double minimum = result.steps.front().trueDistanceM;
  for (const StepRecord& step : result.steps) {
    minimum = std::min(minimum, step.trueDistanceM);
  }
  return minimum;
}

double maxBearingWhileFollowingAfter(const ScenarioResult& result, double fromS)
{
  double maximum = 0.0;
  for (const StepRecord& step : result.steps) {
    if (step.tS >= fromS && step.state == S::Following) {
      maximum = std::max(maximum, std::abs(step.trueBearingDeg));
    }
  }
  return maximum;
}

std::vector<double> distancesBetween(const ScenarioResult& result, double fromS, double toS)
{
  std::vector<double> distances;
  for (const StepRecord& step : result.steps) {
    if (step.tS >= fromS && step.tS <= toS) {
      distances.push_back(step.trueDistanceM);
    }
  }
  return distances;
}

// Every scenario must respect the command envelope, and only Following may move the vehicle.
void expectSafeCommands(const ScenarioResult& result, const core::ControlConfig& control)
{
  for (const StepRecord& step : result.steps) {
    bool idleOrNoFc = step.state == S::Idle || step.state == S::NoFc;
    EXPECT_EQ(!step.setpoint, idleOrNoFc) << "t=" << step.tS;
    if (!step.setpoint) {
      continue;
    }
    EXPECT_LE(std::abs(step.setpoint->vx), control.vxMax + 1e-9) << "t=" << step.tS;
    EXPECT_LE(std::abs(step.setpoint->yawRate), core::degToRad(control.yawRateMaxDps) + 1e-9) << "t=" << step.tS;
    if (step.state != S::Following) {
      EXPECT_EQ(step.setpoint->vx, 0.0) << "t=" << step.tS;
      EXPECT_EQ(step.setpoint->yawRate, 0.0) << "t=" << step.tS;
    }
  }
}

class ScenarioTest : public ::testing::Test {
protected:
  ScenarioResult run(const SimTarget& target, double durationS)
  {
    this->options.durationS = durationS;
    ScenarioResult result = runScenario(this->config, this->camera, this->mount, target, this->options);
    expectSafeCommands(result, this->config.control);
    return result;
  }

  core::FisheyeKbModel camera{core::nominalFisheye(640, 480, 160.0)};
  core::CameraMount mount{};
  core::Config config{};
  ScenarioOptions options{};  // engage at 1 s, vehicle 2 m up facing north
};

}  // namespace

TEST_F(ScenarioTest, StationaryTargetIsHeldWithoutDrift)
{
  // Run: target 3 m ahead, standing still
  ScenarioResult result = run(SimTarget({3.0, 0.0, 0.0}, {TargetHold{30.0}}), 15.0);

  // Assert
  EXPECT_EQ(result.states, (std::vector<S>{S::Idle, S::Locking, S::Following}));
  ASSERT_TRUE(result.lockDistanceM);
  for (const StepRecord& step : result.steps) {
    if (step.state == S::Following) {
      EXPECT_NEAR(step.trueDistanceM, *result.lockDistanceM, 0.15 * *result.lockDistanceM) << "t=" << step.tS;
      EXPECT_LT(std::abs(step.trueBearingDeg), 5.0) << "t=" << step.tS;
    }
  }
}

TEST_F(ScenarioTest, KnownTargetHeightClosesToSetDistance)
{
  // Setup: the target starts 5 m away; its height gives metric distance
  config.estimator.targetHeightM = 1.7;

  // Run
  ScenarioResult result = run(SimTarget({5.0, 0.0, 0.0}, {TargetHold{30.0}}), 15.0);

  // Assert
  EXPECT_EQ(result.states, (std::vector<S>{S::Idle, S::Locking, S::Following}));
  EXPECT_NEAR(result.steps.back().trueDistanceM, config.control.dSet, 0.15 * config.control.dSet);
  EXPECT_GE(minTrueDistance(result), config.control.dMin);
}

TEST_F(ScenarioTest, WalkingAwayIsFollowedWithBoundedLag)
{
  // Run: target walks straight away at 1 m/s from t = 3 s to the end
  ScenarioResult result = run(SimTarget({3.0, 0.0, 0.0}, {TargetHold{2.0}, TargetLine{.durationS = 30.0, .velNorth = 1.0}}), 22.0);

  // Assert
  EXPECT_EQ(result.states, (std::vector<S>{S::Idle, S::Locking, S::Following}));
  std::optional<double> start = followingStart(result);
  ASSERT_TRUE(start);
  ASSERT_TRUE(result.lockDistanceM);
  EXPECT_LT(maxBearingWhileFollowingAfter(result, *start + 3.0), 5.0);

  // Proportional distance control lags by about speed / kD = 2.5 m, and the lag must settle, not grow.
  std::vector<double> steady = distancesBetween(result, 14.0, 22.0);
  auto [low, high] = std::minmax_element(steady.begin(), steady.end());
  double expectedLag = 1.0 / config.control.kD;
  EXPECT_LT(*high, *result.lockDistanceM + 1.5 * expectedLag);
  EXPECT_LT(*high - *low, 0.3);
}

TEST_F(ScenarioTest, CirclingTargetIsTrackedInYaw)
{
  // Run: target walks a 6 m circle at 1 m/s around a point 9 m ahead
  ScenarioResult result =
    run(SimTarget({3.0, 0.0, 0.0}, {TargetHold{2.0}, TargetCircle{.durationS = 60.0, .centerNed = {9.0, 0.0, 0.0}, .speed = 1.0}}), 30.0);

  // Assert
  EXPECT_EQ(result.states, (std::vector<S>{S::Idle, S::Locking, S::Following}));
  std::optional<double> start = followingStart(result);
  ASSERT_TRUE(start);
  EXPECT_LT(maxBearingWhileFollowingAfter(result, *start + 3.0), 15.0);
  EXPECT_GE(minTrueDistance(result), config.control.dMin);
}

TEST_F(ScenarioTest, StopAndGoSettlesBackAtLockDistance)
{
  // Run: walk 5 s, stop 5 s, walk 5 s, stop until the end
  SimTarget target({3.0, 0.0, 0.0},
                   {TargetHold{2.0},
                    TargetLine{.durationS = 5.0, .velNorth = 0.7},
                    TargetHold{5.0},
                    TargetLine{.durationS = 5.0, .velNorth = 0.7},
                    TargetHold{30.0}});
  ScenarioResult result = run(target, 26.0);

  // Assert
  EXPECT_EQ(result.states, (std::vector<S>{S::Idle, S::Locking, S::Following}));
  ASSERT_TRUE(result.lockDistanceM);
  EXPECT_GE(minTrueDistance(result), config.control.dMin);
  EXPECT_NEAR(result.steps.back().trueDistanceM, *result.lockDistanceM, 0.15 * *result.lockDistanceM);
}

TEST_F(ScenarioTest, ShortOcclusionRecoversToFollowing)
{
  ScenarioResult result = run(SimTarget({3.0, 0.0, 0.0}, {TargetHold{30.0}}, {OcclusionWindow{.startS = 5.0, .endS = 6.0}}), 12.0);

  EXPECT_EQ(result.states, (std::vector<S>{S::Idle, S::Locking, S::Following, S::Lost, S::Following}));
}

TEST_F(ScenarioTest, LongOcclusionEndsInHold)
{
  ScenarioResult result = run(SimTarget({3.0, 0.0, 0.0}, {TargetHold{30.0}}, {OcclusionWindow{.startS = 5.0, .endS = 10.0}}), 14.0);

  EXPECT_EQ(result.states, (std::vector<S>{S::Idle, S::Locking, S::Following, S::Lost, S::Hold}));
}

TEST_F(ScenarioTest, TargetRunningPastLeavesViewAndEndsInHold)
{
  // Setup: the target starts 8 m ahead and 1 m right, inside the lock box, is followed,
  // then runs past the vehicle at 5 m/s. Its bearing swings far faster than the 45 deg/s
  // yaw limit, so it eventually leaves the image.
  SimTarget target({8.0, 1.0, 0.0}, {TargetHold{2.0}, TargetLine{.durationS = 3.0, .velNorth = -5.0}});

  // Run
  ScenarioResult result = run(target, 12.0);

  // Assert: Following is reached before the target is lost, and the run ends in Hold.
  // Lost/Following may flicker in between as the target crosses the edge of the view.
  auto firstFollowing = std::find(result.states.begin(), result.states.end(), S::Following);
  auto firstLost = std::find(result.states.begin(), result.states.end(), S::Lost);
  ASSERT_NE(firstFollowing, result.states.end());
  ASSERT_NE(firstLost, result.states.end());
  EXPECT_LT(firstFollowing, firstLost);
  EXPECT_EQ(result.states.back(), S::Hold);
}

TEST_F(ScenarioTest, FastSidewaysDashIsStillFollowed)
{
  // Run: 8 m/s sideways for 1.5 s. Tracking survives because the bearing stays inside the 128 deg view.
  ScenarioResult result = run(SimTarget({3.0, 0.0, 0.0}, {TargetHold{3.0}, TargetLine{.durationS = 1.5, .velEast = 8.0}}), 12.0);

  EXPECT_EQ(result.states, (std::vector<S>{S::Idle, S::Locking, S::Following}));
}

TEST_F(ScenarioTest, LockMissNeverMoves)
{
  // Run: target 53 deg to the right, outside the center lock box
  ScenarioResult result = run(SimTarget({3.0, 4.0, 0.0}, {TargetHold{30.0}}), 8.0);

  // Assert: expectSafeCommands already checked that every setpoint was zero
  EXPECT_EQ(result.states, (std::vector<S>{S::Idle, S::Locking, S::Lost, S::Hold}));
}

TEST_F(ScenarioTest, YawOnlyStageNeverCommandsForwardSpeed)
{
  // Setup: bring-up stage 2
  config.control.enableVx = false;

  // Run
  ScenarioResult result =
    run(SimTarget({3.0, 0.0, 0.0}, {TargetHold{2.0}, TargetCircle{.durationS = 60.0, .centerNed = {9.0, 0.0, 0.0}, .speed = 1.0}}), 15.0);

  // Assert
  EXPECT_EQ(result.states, (std::vector<S>{S::Idle, S::Locking, S::Following}));
  std::optional<double> start = followingStart(result);
  ASSERT_TRUE(start);
  EXPECT_LT(maxBearingWhileFollowingAfter(result, *start + 3.0), 15.0);
  for (const StepRecord& step : result.steps) {
    if (step.setpoint) {
      EXPECT_EQ(step.setpoint->vx, 0.0) << "t=" << step.tS;
    }
  }
}
