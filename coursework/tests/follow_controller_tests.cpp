#include <gtest/gtest.h>

#include "control/FollowController.h"
#include "models/Angles.h"

using namespace follow::control;
using namespace follow::models;

namespace {

constexpr double kDt = 0.05;

TargetState relative(double ratio, double bearingDeg = 0.0)
{
  return {.valid = true, .bearingRad = degToRad(bearingDeg), .ratio = ratio};
}

TargetState measured(double distanceM, double bearingDeg = 0.0)
{
  return {.valid = true, .bearingRad = degToRad(bearingDeg), .distanceM = distanceM, .source = DistanceSource::KnownSize};
}

VelocityCmd settle(FollowController& controller, const TargetState& target, int steps = 60)
{
  VelocityCmd cmd{};
  for (int i = 0; i < steps; ++i) {
    cmd = controller.update(target, kDt);
  }
  return cmd;
}

}  // namespace

TEST(FollowController, CenteredAtLockDistanceCommandsNothing)
{
  FollowController controller(ControlConfig{});
  VelocityCmd cmd = controller.update(relative(1.0), kDt);
  EXPECT_DOUBLE_EQ(cmd.vx, 0.0);
  EXPECT_DOUBLE_EQ(cmd.yawRate, 0.0);
}

TEST(FollowController, YawRateIsProportionalToBearing)
{
  FollowController controller(ControlConfig{});
  VelocityCmd cmd = controller.update(relative(1.0, 10.0), kDt);
  EXPECT_NEAR(cmd.yawRate, 1.5 * degToRad(10.0), 1e-12);
}

TEST(FollowController, YawRateIsClamped)
{
  FollowController controller(ControlConfig{});
  EXPECT_NEAR(controller.update(relative(1.0, 60.0), kDt).yawRate, degToRad(45.0), 1e-12);
  EXPECT_NEAR(controller.update(relative(1.0, -60.0), kDt).yawRate, -degToRad(45.0), 1e-12);
}

TEST(FollowController, SmallBearingIsInsideDeadband)
{
  FollowController controller(ControlConfig{});
  EXPECT_DOUBLE_EQ(controller.update(relative(1.0, 1.5), kDt).yawRate, 0.0);
}

TEST(FollowController, ForwardSpeedRampsAtSlewLimit)
{
  // Setup: ratio 2 means 3 m too far, so the unlimited command is 0.4 * 3 = 1.2 m/s
  FollowController controller(ControlConfig{});

  // Run
  VelocityCmd first = controller.update(relative(2.0), kDt);
  VelocityCmd settled = settle(controller, relative(2.0));

  // Assert
  EXPECT_NEAR(first.vx, 1.5 * kDt, 1e-12);
  EXPECT_NEAR(settled.vx, 1.2, 1e-12);
}

TEST(FollowController, ForwardSpeedIsClamped)
{
  FollowController controller(ControlConfig{});
  EXPECT_NEAR(settle(controller, relative(3.0)).vx, 1.5, 1e-12);
}

TEST(FollowController, HeadingGateScalesForwardSpeed)
{
  // Setup: no slew limit, target 12.5 deg off, half of the 25 deg gate
  ControlConfig config{};
  config.vxSlew = 1000.0;
  FollowController controller(config);

  // Run
  VelocityCmd cmd = controller.update(relative(2.0, 12.5), kDt);

  // Assert
  EXPECT_NEAR(cmd.vx, 1.2 * 0.5, 1e-12);
}

TEST(FollowController, MeasuredDistanceOverridesRatio)
{
  ControlConfig config{};
  config.vxSlew = 1000.0;
  FollowController controller(config);

  VelocityCmd cmd = controller.update(measured(5.0), kDt);

  EXPECT_NEAR(cmd.vx, 0.4 * (5.0 - 3.0), 1e-12);
}

TEST(FollowController, SmallDistanceErrorIsInsideDeadband)
{
  FollowController controller(ControlConfig{});
  EXPECT_DOUBLE_EQ(controller.update(measured(3.2), kDt).vx, 0.0);
}

TEST(FollowController, MinimumDistanceIsHardLimitEvenWhileSlewing)
{
  // Setup: moving forward at 1.2 m/s
  FollowController controller(ControlConfig{});
  ASSERT_NEAR(settle(controller, measured(6.0)).vx, 1.2, 1e-12);

  // Run: target suddenly at 1.4 m, inside d_min
  VelocityCmd cmd = controller.update(measured(1.4), kDt);

  // Assert: the slew limiter alone would still allow 1.125 m/s
  EXPECT_LE(cmd.vx, 0.0);
}

TEST(FollowController, DisabledForwardSpeedKeepsYaw)
{
  ControlConfig config{};
  config.enableVx = false;
  FollowController controller(config);

  VelocityCmd cmd = settle(controller, relative(2.0, 10.0));

  EXPECT_DOUBLE_EQ(cmd.vx, 0.0);
  EXPECT_NEAR(cmd.yawRate, 1.5 * degToRad(10.0), 1e-12);
}

TEST(FollowController, ResetRestartsSlewFromZero)
{
  FollowController controller(ControlConfig{});
  settle(controller, relative(2.0));

  controller.reset();

  EXPECT_NEAR(controller.update(relative(2.0), kDt).vx, 1.5 * kDt, 1e-12);
}

TEST(FollowController, RelativeModeTooCloseAllowsOnlyBackingAway)
{
  // Setup: ratio 0.4 means d_nominal * ratio = 1.2 m, inside d_min (1.5 m)
  FollowController controller(ControlConfig{});

  // Run
  VelocityCmd cmd = controller.update(relative(0.4), kDt);

  // Assert: only backing away (or standing still) is allowed
  EXPECT_LE(cmd.vx, 0.0);
}

TEST(FollowController, HeadingGateAt25DegreesStopsForwardSpeed)
{
  // Setup: no slew limit, ratio 2.0 would otherwise command forward speed
  ControlConfig config{};
  config.vxSlew = 1000.0;
  FollowController controller(config);

  // Run + Assert: at the gate boundary and beyond it, forward speed is fully gated
  EXPECT_DOUBLE_EQ(controller.update(relative(2.0, 25.0), kDt).vx, 0.0);
  EXPECT_DOUBLE_EQ(controller.update(relative(2.0, 40.0), kDt).vx, 0.0);
}
