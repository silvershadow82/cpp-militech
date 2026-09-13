#include <gtest/gtest.h>

#include <cmath>

#include "TestTime.h"
#include "follow/core/Angles.h"
#include "follow/core/TargetEstimator.h"

using namespace follow::core;
using follow::test::at;

namespace {

// Nominal 160 degree fisheye at 640x480: focal length in px.
const double kFocal = 400.0 / degToRad(80.0);

// 40x80 px box centered in the image.
const BBox kCentered{.x = 300.0, .y = 200.0, .w = 40.0, .h = 80.0};

TargetObservation observation(double tSec, const BBox& box)
{
  return {.tFrame = at(tSec), .box = box, .ok = true, .confidence = 1.0};
}

// Samples every 50 ms from 0.5 s to 1.5 s: exactly the 1 s the history keeps.
AttitudeHistory attitudeRamp(double roll, double pitch, double yawRate)
{
  AttitudeHistory history;
  for (int i = 10; i <= 30; ++i) {
    double t = i * 0.05;
    history.push({.t = at(t), .roll = roll, .pitch = pitch, .yaw = yawRate * t});
  }
  return history;
}

BBox scaled(const BBox& box, double factor)
{
  double w = box.w * factor;
  double h = box.h * factor;
  return {.x = box.centerU() - w / 2.0, .y = box.centerV() - h / 2.0, .w = w, .h = h};
}

class TargetEstimatorTest : public ::testing::Test {
protected:
  // The estimator's size measure: angular height of the box.
  double angularSize(const BBox& box) const
  {
    return angleBetween(camera.pixelToRay({box.centerU(), box.y}), camera.pixelToRay({box.centerU(), box.y + box.h}));
  }

  FisheyeKbModel camera{nominalFisheye(640, 480, 160.0)};
  EstimatorConfig config{};
  AttitudeHistory level = attitudeRamp(0.0, 0.0, 0.0);
};

}  // namespace

TEST_F(TargetEstimatorTest, CenteredBoxHasZeroBearingAndUnitRatio)
{
  TargetEstimator estimator(config, camera, {});

  TargetState state = estimator.update(at(1.0), level, observation(0.95, kCentered), std::nullopt);

  ASSERT_TRUE(state.valid);
  EXPECT_NEAR(state.bearingRad, 0.0, 1e-9);
  EXPECT_DOUBLE_EQ(state.ratio, 1.0);
  EXPECT_FALSE(state.distanceM);
  EXPECT_EQ(state.source, DistanceSource::Relative);
}

TEST_F(TargetEstimatorTest, BoxRightOfCenterHasPositiveBearing)
{
  // Setup: box center at u = 480, 160 px right of the principal point
  TargetEstimator estimator(config, camera, {});
  BBox right{.x = 460.0, .y = 200.0, .w = 40.0, .h = 80.0};

  // Run
  TargetState state = estimator.update(at(1.0), level, observation(0.95, right), std::nullopt);

  // Assert: equidistant model, so the angle is 160 px / focal
  ASSERT_TRUE(state.valid);
  EXPECT_NEAR(state.bearingRad, 160.0 / kFocal, 1e-9);
}

TEST_F(TargetEstimatorTest, CameraTiltAndNoseDownPitchCancel)
{
  // Setup: camera tilted up 10 deg on a vehicle pitched 10 deg nose down
  TargetEstimator estimator(config, camera, {.tiltUpDeg = 10.0});
  AttitudeHistory noseDown = attitudeRamp(0.0, degToRad(-10.0), 0.0);
  BBox right{.x = 460.0, .y = 200.0, .w = 40.0, .h = 80.0};

  // Run
  TargetState state = estimator.update(at(1.0), noseDown, observation(0.95, right), std::nullopt);

  // Assert: same bearing as a level, untilted camera
  ASSERT_TRUE(state.valid);
  EXPECT_NEAR(state.bearingRad, 160.0 / kFocal, 1e-9);
}

TEST_F(TargetEstimatorTest, LatencyCompensationSubtractsYawSinceFrame)
{
  // Setup: yawing right at 0.5 rad/s; newest attitude sample at 1.0 s
  AttitudeHistory turning;
  for (int i = 0; i <= 20; ++i) {
    double t = i * 0.05;
    turning.push({.t = at(t), .yaw = 0.5 * t});
  }
  TargetEstimator estimator(config, camera, {});

  // Run: frame captured at 0.8 s shows the target dead ahead
  TargetState state = estimator.update(at(1.0), turning, observation(0.8, kCentered), std::nullopt);

  // Assert: the vehicle turned 0.1 rad right since, so the target is now 0.1 rad left
  ASSERT_TRUE(state.valid);
  EXPECT_NEAR(state.bearingRad, -0.1, 1e-9);
}

TEST_F(TargetEstimatorTest, RatioComparesAngularSizeWithLockReference)
{
  // Setup: no smoothing
  config.emaAlpha = 1.0;
  TargetEstimator estimator(config, camera, {});
  estimator.lock(at(0.5));
  BBox larger = scaled(kCentered, 1.2);

  // Run
  TargetState first = estimator.update(at(0.65), level, observation(0.6, kCentered), std::nullopt);
  TargetState second = estimator.update(at(0.75), level, observation(0.7, larger), std::nullopt);

  // Assert: bigger box means closer target, ratio below 1
  ASSERT_TRUE(first.valid);
  ASSERT_TRUE(second.valid);
  EXPECT_DOUBLE_EQ(first.ratio, 1.0);
  EXPECT_NEAR(second.ratio, angularSize(kCentered) / angularSize(larger), 1e-12);
  EXPECT_LT(second.ratio, 1.0);
}

TEST_F(TargetEstimatorTest, SmoothsSizeOncePerFrame)
{
  // Setup
  TargetEstimator estimator(config, camera, {});
  BBox larger = scaled(kCentered, 1.2);
  estimator.update(at(0.65), level, observation(0.6, kCentered), std::nullopt);

  // Run: the same frame seen twice by the control loop
  estimator.update(at(0.75), level, observation(0.7, larger), std::nullopt);
  TargetState state = estimator.update(at(0.8), level, observation(0.7, larger), std::nullopt);

  // Assert
  double reference = angularSize(kCentered);
  double smoothed = 0.3 * angularSize(larger) + 0.7 * reference;
  ASSERT_TRUE(state.valid);
  EXPECT_NEAR(state.ratio, reference / smoothed, 1e-12);
}

TEST_F(TargetEstimatorTest, MissingObservationIsInvalid)
{
  TargetEstimator estimator(config, camera, {});
  EXPECT_FALSE(estimator.update(at(1.0), level, std::nullopt, std::nullopt).valid);
}

TEST_F(TargetEstimatorTest, TrackerFailureIsInvalid)
{
  TargetEstimator estimator(config, camera, {});
  TargetObservation failed = observation(0.95, kCentered);
  failed.ok = false;
  EXPECT_FALSE(estimator.update(at(1.0), level, failed, std::nullopt).valid);
}

TEST_F(TargetEstimatorTest, LowConfidenceIsInvalid)
{
  TargetEstimator estimator(config, camera, {});
  TargetObservation weak = observation(0.95, kCentered);
  weak.confidence = 0.1;
  EXPECT_FALSE(estimator.update(at(1.0), level, weak, std::nullopt).valid);
}

TEST_F(TargetEstimatorTest, StaleObservationIsInvalid)
{
  TargetEstimator estimator(config, camera, {});
  EXPECT_FALSE(estimator.update(at(1.0), level, observation(0.6, kCentered), std::nullopt).valid);
}

TEST_F(TargetEstimatorTest, BoxTouchingBorderIsInvalid)
{
  TargetEstimator estimator(config, camera, {});
  BBox atEdge{.x = 0.0, .y = 200.0, .w = 40.0, .h = 80.0};
  EXPECT_FALSE(estimator.update(at(1.0), level, observation(0.95, atEdge), std::nullopt).valid);
}

TEST_F(TargetEstimatorTest, FrameCapturedBeforeLockIsIgnored)
{
  TargetEstimator estimator(config, camera, {});
  estimator.lock(at(1.0));
  EXPECT_FALSE(estimator.update(at(1.05), level, observation(0.95, kCentered), std::nullopt).valid);
}

TEST_F(TargetEstimatorTest, AreaJumpInvalidatesOnlyThatFrame)
{
  // Setup: 1.6x linear scale is a 2.56x area jump
  TargetEstimator estimator(config, camera, {});
  BBox jumped = scaled(kCentered, 1.6);

  // Run
  TargetState before = estimator.update(at(0.65), level, observation(0.6, kCentered), std::nullopt);
  TargetState jump = estimator.update(at(0.75), level, observation(0.7, jumped), std::nullopt);
  TargetState after = estimator.update(at(0.85), level, observation(0.8, jumped), std::nullopt);

  // Assert
  EXPECT_TRUE(before.valid);
  EXPECT_FALSE(jump.valid);
  EXPECT_TRUE(after.valid);
}

TEST_F(TargetEstimatorTest, MissingAttitudeIsInvalid)
{
  TargetEstimator estimator(config, camera, {});
  EXPECT_FALSE(estimator.update(at(1.0), AttitudeHistory{}, observation(0.95, kCentered), std::nullopt).valid);
}

TEST_F(TargetEstimatorTest, KnownTargetHeightGivesDistance)
{
  // Setup
  config.emaAlpha = 1.0;
  config.targetHeightM = 1.7;
  TargetEstimator estimator(config, camera, {});

  // Run
  TargetState state = estimator.update(at(1.0), level, observation(0.95, kCentered), std::nullopt);

  // Assert: the 80 px tall centered box spans 80 px / focal radians
  double angularHeight = 80.0 / kFocal;
  ASSERT_TRUE(state.valid);
  ASSERT_TRUE(state.distanceM);
  EXPECT_NEAR(*state.distanceM, 1.7 / (2.0 * std::tan(angularHeight / 2.0)), 1e-9);
  EXPECT_EQ(state.source, DistanceSource::KnownSize);
}

TEST_F(TargetEstimatorTest, RangeUsedWhenCenteredAndInSync)
{
  TargetEstimator estimator(config, camera, {});
  RangeMeasurement range{.t = at(0.97), .rangeM = 4.2, .valid = true};

  TargetState state = estimator.update(at(1.0), level, observation(0.95, kCentered), range);

  ASSERT_TRUE(state.distanceM);
  EXPECT_DOUBLE_EQ(*state.distanceM, 4.2);
  EXPECT_EQ(state.source, DistanceSource::Range);
}

TEST_F(TargetEstimatorTest, RangeIgnoredWhenTargetOffCenter)
{
  TargetEstimator estimator(config, camera, {});
  BBox right{.x = 460.0, .y = 200.0, .w = 40.0, .h = 80.0};
  RangeMeasurement range{.t = at(0.97), .rangeM = 4.2, .valid = true};

  TargetState state = estimator.update(at(1.0), level, observation(0.95, right), range);

  EXPECT_FALSE(state.distanceM);
  EXPECT_EQ(state.source, DistanceSource::Relative);
}

TEST_F(TargetEstimatorTest, RangeIgnoredWhenOutOfSyncWithFrame)
{
  TargetEstimator estimator(config, camera, {});
  RangeMeasurement range{.t = at(0.85), .rangeM = 4.2, .valid = true};

  TargetState state = estimator.update(at(1.0), level, observation(0.95, kCentered), range);

  EXPECT_EQ(state.source, DistanceSource::Relative);
}

TEST_F(TargetEstimatorTest, AcceptedRangeIsHeldFor300Ms)
{
  // Setup
  TargetEstimator estimator(config, camera, {});
  RangeMeasurement range{.t = at(0.97), .rangeM = 4.2, .valid = true};
  estimator.update(at(1.0), level, observation(0.95, kCentered), range);

  // Run
  TargetState held = estimator.update(at(1.2), level, observation(1.15, kCentered), std::nullopt);
  TargetState expired = estimator.update(at(1.4), level, observation(1.35, kCentered), std::nullopt);

  // Assert
  EXPECT_EQ(held.source, DistanceSource::Range);
  EXPECT_EQ(expired.source, DistanceSource::Relative);
}

TEST_F(TargetEstimatorTest, ResetForgetsLastGoodBox)
{
  TargetEstimator estimator(config, camera, {});
  estimator.update(at(1.0), level, observation(0.95, kCentered), std::nullopt);
  ASSERT_TRUE(estimator.lastGoodBox());

  estimator.reset();

  EXPECT_FALSE(estimator.lastGoodBox());
}
