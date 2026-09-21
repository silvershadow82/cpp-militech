#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

#include "sim/KinematicVehicle.h"

using namespace follow;
using namespace follow::sim;

namespace {

constexpr double kDt = 0.01;

void run(KinematicVehicle& vehicle, const std::optional<models::VelocityCmd>& cmd, double seconds)
{
  auto steps = static_cast<int>(std::lround(seconds / kDt));
  for (int i = 0; i < steps; ++i) {
    vehicle.step(cmd, kDt);
  }
}

}  // namespace

TEST(KinematicVehicle, ForwardSpeedFollowsCommandWithLag)
{
  KinematicVehicle vehicle(Pose{}, {.tauS = 0.3});

  run(vehicle, models::VelocityCmd{.vx = 1.0}, 0.3);
  double afterTau = vehicle.forwardSpeed();
  run(vehicle, models::VelocityCmd{.vx = 1.0}, 2.7);

  // (1 - 1/30)^30 = 0.362, so 63.8% of the step after one time constant
  EXPECT_NEAR(afterTau, 0.638, 0.005);
  EXPECT_GT(vehicle.forwardSpeed(), 0.99);
}

TEST(KinematicVehicle, MovesAlongHeading)
{
  KinematicVehicle vehicle(Pose{.yaw = std::numbers::pi / 2.0}, {});

  run(vehicle, models::VelocityCmd{.vx = 1.0}, 5.0);

  EXPECT_NEAR(vehicle.pose().positionNed.x, 0.0, 1e-9);
  EXPECT_GT(vehicle.pose().positionNed.y, 4.0);
}

TEST(KinematicVehicle, YawRateIntegratesIntoYaw)
{
  KinematicVehicle vehicle(Pose{}, {.tauS = 0.3});

  run(vehicle, models::VelocityCmd{.yawRate = 0.5}, 4.0);

  // 0.5 rad/s for 4 s, minus about one time constant of lag
  EXPECT_NEAR(vehicle.pose().yaw, 0.5 * (4.0 - 0.3), 0.02);
}

TEST(KinematicVehicle, AcceleratingPitchesNoseDown)
{
  KinematicVehicle vehicle(Pose{}, {});
  vehicle.step(models::VelocityCmd{.vx = 1.0}, kDt);
  EXPECT_LT(vehicle.pose().pitch, 0.0);
}

TEST(KinematicVehicle, NoCommandSlowsToStop)
{
  KinematicVehicle vehicle(Pose{}, {});
  run(vehicle, models::VelocityCmd{.vx = 1.0}, 3.0);

  run(vehicle, std::nullopt, 3.0);

  EXPECT_LT(vehicle.forwardSpeed(), 0.01);
}

TEST(KinematicVehicle, ZeroTimeStepLeavesStateUnchanged)
{
  // Setup
  KinematicVehicle vehicle(Pose{}, {});
  run(vehicle, models::VelocityCmd{.vx = 1.0}, 1.0);
  Pose before = vehicle.pose();
  double speedBefore = vehicle.forwardSpeed();

  // Run
  vehicle.step(models::VelocityCmd{.vx = 1.0}, 0.0);

  // Assert
  EXPECT_DOUBLE_EQ(vehicle.forwardSpeed(), speedBefore);
  EXPECT_DOUBLE_EQ(vehicle.pose().positionNed.x, before.positionNed.x);
  EXPECT_DOUBLE_EQ(vehicle.pose().pitch, before.pitch);
  EXPECT_FALSE(std::isnan(vehicle.pose().pitch));
}

TEST(KinematicVehicle, StepLongerThanTimeConstantDoesNotOvershoot)
{
  // Setup: 0.3 s time constant stepped with a single 1 s step
  KinematicVehicle vehicle(Pose{}, {.tauS = 0.3});

  // Run
  vehicle.step(models::VelocityCmd{.vx = 1.0, .yawRate = 0.5}, 1.0);

  // Assert
  EXPECT_DOUBLE_EQ(vehicle.forwardSpeed(), 1.0);
  EXPECT_DOUBLE_EQ(vehicle.yawRate(), 0.5);
}
