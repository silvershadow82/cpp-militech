#include <gtest/gtest.h>

#include "TestTime.h"
#include "follow/core/Core.h"

using namespace follow::core;
using follow::test::at;

namespace {

class CoreTest : public ::testing::Test {
protected:
  // One 20 Hz control step at tSec with a fresh heartbeat and level attitude.
  Outputs step(double tSec, uint32_t mode, const std::optional<TargetObservation>& target = std::nullopt)
  {
    vehicle.lastHeartbeat = at(tSec);
    vehicle.customMode = mode;
    vehicle.attitude.push({.t = at(tSec)});
    return core.step({.now = at(tSec), .vehicle = vehicle, .target = target});
  }

  static TargetObservation seen(double tSec, const BBox& box) { return {.tFrame = at(tSec), .box = box, .ok = true, .confidence = 1.0}; }

  // LOITER at 0.0, GUIDED at 0.05: returns the Locking step.
  Outputs engage()
  {
    step(0.0, kModeLoiter);
    return step(0.05, kModeGuided);
  }

  FisheyeKbModel camera{nominalFisheye(640, 480, 160.0)};
  Core core{Config{}, camera, CameraMount{}};
  VehicleState vehicle{};
  const BBox rightOfCenter{.x = 380.0, .y = 200.0, .w = 40.0, .h = 80.0};
};

}  // namespace

TEST_F(CoreTest, IdleSendsNothing)
{
  Outputs out = step(0.0, kModeLoiter);
  EXPECT_EQ(out.state, State::Idle);
  EXPECT_FALSE(out.setpoint);
  EXPECT_EQ(out.tracker.kind, TrackerRequestKind::None);
}

TEST_F(CoreTest, EngagingRequestsCenterLockAndHoldsStill)
{
  Outputs out = engage();

  EXPECT_EQ(out.state, State::Locking);
  EXPECT_EQ(out.tracker.kind, TrackerRequestKind::LockCenter);
  // 20% of 480 px = 96 px square in the middle of 640x480
  EXPECT_DOUBLE_EQ(out.tracker.hint.x, 272.0);
  EXPECT_DOUBLE_EQ(out.tracker.hint.y, 192.0);
  EXPECT_DOUBLE_EQ(out.tracker.hint.w, 96.0);
  ASSERT_TRUE(out.setpoint);
  EXPECT_DOUBLE_EQ(out.setpoint->vx, 0.0);
  EXPECT_DOUBLE_EQ(out.setpoint->yawRate, 0.0);
}

TEST_F(CoreTest, ValidTargetAfterLockStartsFollowing)
{
  engage();

  Outputs out = step(0.1, kModeGuided, seen(0.08, rightOfCenter));

  EXPECT_EQ(out.state, State::Following);
  ASSERT_TRUE(out.setpoint);
  EXPECT_GT(out.setpoint->yawRate, 0.0);
  ASSERT_TRUE(out.overlay.targetBox);
  EXPECT_DOUBLE_EQ(out.overlay.targetBox->x, rightOfCenter.x);
}

TEST_F(CoreTest, LosingTargetRequestsReacquireAtLastGoodBox)
{
  engage();
  step(0.1, kModeGuided, seen(0.08, rightOfCenter));

  TargetObservation failed = seen(0.13, rightOfCenter);
  failed.ok = false;
  Outputs out = step(0.15, kModeGuided, failed);

  EXPECT_EQ(out.state, State::Lost);
  EXPECT_EQ(out.tracker.kind, TrackerRequestKind::Reacquire);
  EXPECT_DOUBLE_EQ(out.tracker.hint.x, rightOfCenter.x);
  ASSERT_TRUE(out.setpoint);
  EXPECT_DOUBLE_EQ(out.setpoint->vx, 0.0);
}

TEST_F(CoreTest, LockMissNeverRequestsReacquire)
{
  engage();
  Outputs out{};
  for (double t = 0.1; t < 1.2; t += 0.05) {
    out = step(t, kModeGuided);
    EXPECT_NE(out.tracker.kind, TrackerRequestKind::Reacquire);
  }
  EXPECT_EQ(out.state, State::Lost);
}

TEST_F(CoreTest, LeavingGuidedRequestsUnlockAndStopsSetpoints)
{
  engage();
  step(0.1, kModeGuided, seen(0.08, rightOfCenter));

  Outputs out = step(0.15, kModeLoiter, seen(0.13, rightOfCenter));

  EXPECT_EQ(out.state, State::Idle);
  EXPECT_EQ(out.tracker.kind, TrackerRequestKind::Unlock);
  EXPECT_FALSE(out.setpoint);
}

TEST_F(CoreTest, FrameFromBeforeLockDoesNotStartFollowing)
{
  step(0.0, kModeLoiter);
  step(0.05, kModeGuided);

  Outputs out = step(0.1, kModeGuided, seen(0.02, rightOfCenter));

  EXPECT_EQ(out.state, State::Locking);
}
