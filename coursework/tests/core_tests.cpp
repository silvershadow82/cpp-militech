#include <gtest/gtest.h>

#include "TestTime.h"
#include "control/Core.h"
#include "models/FisheyeKbModel.h"

using namespace follow::control;
using namespace follow::models;
using follow::test::at;

namespace {

class CoreTest : public ::testing::Test {
protected:
  // One 20 Hz control step at tSec with a fresh heartbeat and level attitude. `refreshHeartbeat = false`
  // leaves vehicle.lastHeartbeat untouched, simulating a silent FC.
  Outputs step(double tSec, uint32_t mode, const std::optional<TargetObservation>& target = std::nullopt, bool refreshHeartbeat = true)
  {
    if (refreshHeartbeat) {
      this->vehicle.lastHeartbeat = at(tSec);
    }
    this->vehicle.customMode = mode;
    this->vehicle.attitude.push({.t = at(tSec)});
    return this->core.step({.now = at(tSec), .vehicle = this->vehicle, .target = target});
  }

  static TargetObservation seen(double tSec, const BBox& box) { return {.tFrame = at(tSec), .box = box, .ok = true, .confidence = 1.0}; }

  // LOITER at 0.0, GUIDED at 0.05: returns the Locking step.
  Outputs engage()
  {
    this->step(0.0, kModeLoiter);
    return this->step(0.05, kModeGuided);
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

TEST_F(CoreTest, HeartbeatLossUnlocksAndStopsSetpointsThenRecoversToIdle)
{
  // Setup: engage and reach Following, same as ValidTargetAfterLockStartsFollowing
  engage();
  step(0.1, kModeGuided, seen(0.08, rightOfCenter));

  // Run: the heartbeat is left stale past the 2 s FC timeout while attitude keeps arriving
  Outputs stale = step(2.2, kModeGuided, std::nullopt, /*refreshHeartbeat=*/false);

  // Assert: the FC is presumed lost, the tracker is unlocked, and nothing is commanded
  EXPECT_EQ(stale.state, State::NoFc);
  EXPECT_EQ(stale.tracker.kind, TrackerRequestKind::Unlock);
  EXPECT_FALSE(stale.setpoint);

  // Run: the heartbeat resumes in GUIDED
  Outputs recovered = step(2.25, kModeGuided);

  // Assert: recovers to Idle (GUIDED alone does not re-engage after an FC loss)
  EXPECT_EQ(recovered.state, State::Idle);
  EXPECT_FALSE(recovered.setpoint);
}

TEST_F(CoreTest, ReacquiringTargetRestartsForwardSpeedFromZero)
{
  // Setup: a local Core with target.height_m set so a centered ~80 px tall box reads as
  // roughly 6 m away, well past d_set (3 m), so Following commands sustained forward speed.
  Config config{};
  config.estimator.targetHeightM = 1.7;
  Core localCore(config, camera, CameraMount{});
  VehicleState localVehicle{};
  const BBox farBox{.x = 300.0, .y = 200.0, .w = 40.0, .h = 80.0};
  const double dt = 0.05;

  auto localStep = [&](double tSec, uint32_t mode, const std::optional<TargetObservation>& target = std::nullopt) {
    localVehicle.lastHeartbeat = at(tSec);
    localVehicle.customMode = mode;
    localVehicle.attitude.push({.t = at(tSec)});
    return localCore.step({.now = at(tSec), .vehicle = localVehicle, .target = target});
  };

  // Run: engage, lock, then follow the far target until forward speed clearly exceeds one slew step
  localStep(0.0, kModeLoiter);
  localStep(0.05, kModeGuided);
  Outputs out{};
  double t = 0.1;
  for (; t < 2.0; t += dt) {
    out = localStep(t, kModeGuided, seen(t - 0.02, farBox));
    if (out.state == State::Following && out.setpoint && out.setpoint->vx > 10.0 * config.control.vxSlew * dt) {
      break;
    }
  }
  ASSERT_EQ(out.state, State::Following);
  ASSERT_TRUE(out.setpoint);
  ASSERT_GT(out.setpoint->vx, config.control.vxSlew * dt + 1e-9);

  // Run: lose the target for one step
  TargetObservation failed = seen(t - 0.02, farBox);
  failed.ok = false;
  t += dt;
  Outputs lost = localStep(t, kModeGuided, failed);
  ASSERT_EQ(lost.state, State::Lost);
  ASSERT_TRUE(lost.setpoint);
  EXPECT_DOUBLE_EQ(lost.setpoint->vx, 0.0);

  // Assert: reacquiring the target restarts the forward-speed slew limiter from zero
  t += dt;
  Outputs reacquired = localStep(t, kModeGuided, seen(t - 0.02, farBox));
  EXPECT_EQ(reacquired.state, State::Following);
  ASSERT_TRUE(reacquired.setpoint);
  EXPECT_LE(reacquired.setpoint->vx, config.control.vxSlew * dt + 1e-9);
}
