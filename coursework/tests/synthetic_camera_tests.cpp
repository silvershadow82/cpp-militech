#include <gtest/gtest.h>

#include <cmath>

#include "TestTime.h"
#include "follow/core/Angles.h"
#include "follow/core/TargetEstimator.h"
#include "follow/sim/SyntheticCamera.h"

using namespace follow;
using namespace follow::sim;
using follow::test::at;

namespace {

class SyntheticCameraTest : public ::testing::Test {
protected:
  SyntheticCameraTest() { this->config.pixelNoiseSigma = 0.0; }

  core::FisheyeKbModel camera{core::nominalFisheye(640, 480, 160.0)};
  SyntheticCameraConfig config{};
  const Pose hover{.positionNed = {0.0, 0.0, -2.0}};
  const core::BBox lockBox{.x = 272.0, .y = 192.0, .w = 96.0, .h = 96.0};
  const core::TrackerRequest lockCenter{.kind = core::TrackerRequestKind::LockCenter, .hint = lockBox};
};

}  // namespace

TEST_F(SyntheticCameraTest, TargetDeadAheadIsHorizontallyCentered)
{
  SyntheticCamera cam(camera, {}, config);

  std::optional<core::BBox> box = cam.project(hover, {3.0, 0.0, 0.0}, 1.7, 0.5);

  ASSERT_TRUE(box);
  EXPECT_NEAR(box->centerU(), 320.0, 1e-6);
  // Camera 2 m up looks down at a 1.7 m target: the box sits below the image center.
  EXPECT_GT(box->y, 240.0);
}

TEST_F(SyntheticCameraTest, TargetToTheRightProjectsRightOfCenter)
{
  SyntheticCamera cam(camera, {}, config);
  std::optional<core::BBox> box = cam.project(hover, {3.0, 1.0, 0.0}, 1.7, 0.5);
  ASSERT_TRUE(box);
  EXPECT_GT(box->centerU(), 320.0);
}

TEST_F(SyntheticCameraTest, TargetBehindIsNotVisible)
{
  SyntheticCamera cam(camera, {}, config);
  EXPECT_FALSE(cam.project(hover, {-3.0, 0.0, 0.0}, 1.7, 0.5));
}

TEST_F(SyntheticCameraTest, ObservationArrivesAfterLatency)
{
  // Setup
  SyntheticCamera cam(camera, {}, config);
  SimTarget target({3.0, 0.0, 0.0}, {TargetHold{10.0}});
  cam.handle(lockCenter, at(0.0));

  // Run
  std::optional<core::TargetObservation> early = cam.step(at(0.0), hover, target, 0.0);
  std::optional<core::TargetObservation> stillEarly = cam.step(at(0.05), hover, target, 0.05);
  std::optional<core::TargetObservation> delivered = cam.step(at(0.1), hover, target, 0.1);

  // Assert: 80 ms latency delivers the 0.0 s frame at 0.1 s
  EXPECT_FALSE(early);
  EXPECT_FALSE(stillEarly);
  ASSERT_TRUE(delivered);
  EXPECT_EQ(delivered->tFrame, at(0.0));
  EXPECT_TRUE(delivered->ok);
}

TEST_F(SyntheticCameraTest, LockMissWhenTargetOutsideLockBox)
{
  SyntheticCamera cam(camera, {}, config);
  SimTarget target({3.0, 4.0, 0.0}, {TargetHold{10.0}});
  cam.handle(lockCenter, at(0.0));

  cam.step(at(0.0), hover, target, 0.0);
  std::optional<core::TargetObservation> obs = cam.step(at(0.1), hover, target, 0.1);

  ASSERT_TRUE(obs);
  EXPECT_FALSE(obs->ok);
}

TEST_F(SyntheticCameraTest, OcclusionLosesLockAndReacquireRestoresIt)
{
  // Setup: hidden between 0.2 and 0.4 s
  SyntheticCamera cam(camera, {}, config);
  SimTarget target({3.0, 0.0, 0.0}, {TargetHold{10.0}}, {OcclusionWindow{.startS = 0.2, .endS = 0.4}});
  std::optional<core::BBox> truth = cam.project(hover, {3.0, 0.0, 0.0}, 1.7, 0.5);
  ASSERT_TRUE(truth);
  cam.handle(lockCenter, at(0.0));

  // Run
  std::optional<core::TargetObservation> obs;
  for (int i = 0; i <= 6; ++i) {
    obs = cam.step(at(i * 0.05), hover, target, i * 0.05);
  }
  ASSERT_TRUE(obs);
  EXPECT_FALSE(obs->ok);  // frame at 0.2 s, occluded

  cam.handle({.kind = core::TrackerRequestKind::Reacquire, .hint = *truth}, at(0.3));
  for (int i = 7; i <= 20; ++i) {
    obs = cam.step(at(i * 0.05), hover, target, i * 0.05);
  }

  // Assert
  ASSERT_TRUE(obs);
  EXPECT_TRUE(obs->ok);
}

TEST_F(SyntheticCameraTest, OffCenterTargetKeepsAngularSizeButNotPixelSize)
{
  // Setup: the same target 4 m away, dead ahead and 45 deg to the right
  SyntheticCamera cam(camera, {}, config);
  double side = 4.0 * std::cos(core::degToRad(45.0));
  std::optional<core::BBox> ahead = cam.project(hover, {4.0, 0.0, 0.0}, 1.7, 0.5);
  std::optional<core::BBox> offCenter = cam.project(hover, {side, side, 0.0}, 1.7, 0.5);
  ASSERT_TRUE(ahead);
  ASSERT_TRUE(offCenter);

  core::AttitudeHistory level;
  level.push({.t = at(0.0)});
  core::EstimatorConfig estimatorConfig{};
  estimatorConfig.emaAlpha = 1.0;
  estimatorConfig.maxAreaJump = 10.0;
  core::TargetEstimator estimator(estimatorConfig, camera, {});

  // Run
  core::TargetState first = estimator.update(
    at(0.05), level, core::TargetObservation{.tFrame = at(0.0), .box = *ahead, .ok = true, .confidence = 1.0}, std::nullopt);
  core::TargetState second = estimator.update(
    at(0.1), level, core::TargetObservation{.tFrame = at(0.05), .box = *offCenter, .ok = true, .confidence = 1.0}, std::nullopt);

  // Assert: the fisheye squeezes the box in pixels, but the estimated distance ratio stays ~1
  ASSERT_TRUE(first.valid);
  ASSERT_TRUE(second.valid);
  EXPECT_NEAR(second.ratio, 1.0, 0.05);
  EXPECT_GT(std::abs(offCenter->area() / ahead->area() - 1.0), 0.1);
}

TEST_F(SyntheticCameraTest, UnlockStopsValidObservations)
{
  SyntheticCamera cam(camera, {}, config);
  SimTarget target({3.0, 0.0, 0.0}, {TargetHold{10.0}});
  cam.handle(lockCenter, at(0.0));
  cam.step(at(0.0), hover, target, 0.0);

  cam.handle({.kind = core::TrackerRequestKind::Unlock}, at(0.05));
  std::optional<core::TargetObservation> obs;
  for (int i = 1; i <= 4; ++i) {
    obs = cam.step(at(i * 0.05), hover, target, i * 0.05);
  }

  ASSERT_TRUE(obs);
  EXPECT_FALSE(obs->ok);
}
