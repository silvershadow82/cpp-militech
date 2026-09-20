#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

#include "TestTime.h"
#include "providers/SimVision.h"
#include "util/Channels.h"

using namespace follow;
using namespace follow::providers;
using namespace follow::util;
using follow::test::at;

namespace {

class SimVisionTest : public ::testing::Test {
protected:
  // Publishes the vehicle state the MAVLink thread would: heartbeat, attitude and (optionally) position.
  void publishVehicle(double tSec, const models::Vec3& positionNed, double yaw, bool withPosition = true)
  {
    control::VehicleState vehicle;
    vehicle.lastHeartbeat = at(tSec);
    vehicle.customMode = models::kModeGuided;
    vehicle.attitude.push({.t = at(tSec), .yaw = yaw});
    if (withPosition) {
      vehicle.position = models::LocalPositionNed{.t = at(tSec), .position = positionNed};
    }
    channels.vehicle.write(vehicle, at(tSec));
  }

  void lockCenter()
  {
    channels.trackerRequests.push({.kind = control::TrackerRequestKind::LockCenter, .hint = {272.0, 192.0, 96.0, 96.0}});
  }

  models::FisheyeKbModel camera{models::nominalFisheye(640, 480, 160.0)};
  Channels channels;
  config::TargetScript script{.start = {3.0, 0.0, 0.0}, .motions = {sim::TargetHold{30.0}}};
  SimVision vision{camera, models::CameraMount{}, sim::SyntheticCameraConfig{.pixelNoiseSigma = 0.0}, script, 5.0, channels};
};

}  // namespace

TEST_F(SimVisionTest, NothingIsPublishedBeforeEngage)
{
  publishVehicle(0.0, {0.0, 0.0, -2.0}, 0.0);

  vision.iterate(at(0.0));
  vision.iterate(at(0.2));

  EXPECT_FALSE(channels.observation.read());
  EXPECT_FALSE(channels.truth.read());
}

TEST_F(SimVisionTest, LockCenterPlacesTheTargetAheadOfTheVehicle)
{
  // Setup: vehicle 2 m up at (10, 5) facing east
  publishVehicle(0.0, {10.0, 5.0, -2.0}, std::numbers::pi / 2.0);
  lockCenter();

  // Run: long enough for one frame to pass the 80 ms latency
  for (double t = 0.0; t <= 0.2; t += 0.01) {
    vision.iterate(at(t));
  }

  // Assert: the target stands 3 m ahead, centered in the image
  auto observation = channels.observation.read();
  ASSERT_TRUE(observation);
  EXPECT_TRUE(observation->value.ok);
  EXPECT_NEAR(observation->value.box.centerU(), 320.0, 1.0);
  auto truth = channels.truth.read();
  ASSERT_TRUE(truth);
  EXPECT_NEAR(truth->value.bearingDeg, 0.0, 1e-6);
  EXPECT_NEAR(truth->value.distanceM, std::hypot(3.0, 2.0 - 0.85), 1e-6);
}

TEST_F(SimVisionTest, LockCenterBeforePositionIsLatchedAndPlacedWhenPoseArrives)
{
  // Setup: the first LockCenter arrives before any LOCAL_POSITION_NED
  publishVehicle(0.0, {}, 0.0, false);
  lockCenter();
  vision.iterate(at(0.0));

  // Assert: nothing placed yet, no pose to place against
  EXPECT_FALSE(channels.observation.read());

  // Run: position arrives later; the latched LockCenter should engage on the first pose
  publishVehicle(0.1, {0.0, 0.0, -2.0}, 0.0);
  for (double t = 0.1; t <= 0.5; t += 0.05) {
    vision.iterate(at(t));
  }

  // Assert: the target was placed once a pose became available
  EXPECT_TRUE(channels.observation.read());
}

TEST_F(SimVisionTest, LockCenterWithoutPositionThenLaterPoseProducesObservations)
{
  // Setup: no vehicle state at all yet, so vehiclePose() returns nullopt
  lockCenter();
  vision.iterate(at(0.0));

  // Assert: nothing published, nothing engaged
  EXPECT_FALSE(channels.observation.read());
  EXPECT_FALSE(channels.truth.read());

  // Run: pose arrives well after the LockCenter request
  publishVehicle(0.2, {0.0, 0.0, -2.0}, 0.0);
  for (double t = 0.2; t <= 0.4; t += 0.01) {
    vision.iterate(at(t));
  }

  // Assert: the latched request engaged once the pose showed up
  EXPECT_TRUE(channels.observation.read());
  EXPECT_TRUE(channels.truth.read());
}

TEST_F(SimVisionTest, FinishesScenarioDurationAfterEngage)
{
  publishVehicle(0.0, {0.0, 0.0, -2.0}, 0.0);
  lockCenter();
  vision.iterate(at(1.0));

  vision.iterate(at(5.9));
  EXPECT_FALSE(vision.finished());
  vision.iterate(at(6.0));
  EXPECT_TRUE(vision.finished());
}

TEST(SimVisionPoseTest, PositionIsExtrapolatedAtMost200Ms)
{
  // Setup
  control::VehicleState vehicle;
  vehicle.attitude.push({.t = at(1.0), .roll = 0.1, .pitch = 0.2, .yaw = 0.3});
  vehicle.position = models::LocalPositionNed{.t = at(1.0), .position = {1.0, 2.0, -2.0}, .velocity = {1.0, 0.0, 0.0}};

  // Run
  auto soon = SimVision::vehiclePose(vehicle, at(1.1));
  auto late = SimVision::vehiclePose(vehicle, at(2.0));

  // Assert
  ASSERT_TRUE(soon);
  EXPECT_NEAR(soon->positionNed.x, 1.1, 1e-9);
  EXPECT_EQ(soon->yaw, 0.3);
  ASSERT_TRUE(late);
  EXPECT_NEAR(late->positionNed.x, 1.2, 1e-9);
  EXPECT_FALSE(SimVision::vehiclePose(control::VehicleState{}, at(1.0)));
}
