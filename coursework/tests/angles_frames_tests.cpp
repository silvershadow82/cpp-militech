#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

#include "follow/core/Angles.h"
#include "follow/core/Frames.h"

using namespace follow::core;
using std::numbers::pi;

namespace {

void expectVecNear(const Vec3& actual, const Vec3& expected, double tolerance = 1e-9)
{
  EXPECT_NEAR(actual.x, expected.x, tolerance);
  EXPECT_NEAR(actual.y, expected.y, tolerance);
  EXPECT_NEAR(actual.z, expected.z, tolerance);
}

}  // namespace

TEST(Angles, WrapPiKeepsHalfOpenRange)
{
  EXPECT_NEAR(wrapPi(0.0), 0.0, 1e-12);
  EXPECT_NEAR(wrapPi(3.0 * pi / 2.0), -pi / 2.0, 1e-12);
  EXPECT_NEAR(wrapPi(-3.0 * pi / 2.0), pi / 2.0, 1e-12);
  EXPECT_NEAR(wrapPi(-pi), pi, 1e-12);
  EXPECT_NEAR(wrapPi(5.0 * pi), pi, 1e-9);
}

TEST(Angles, ConvertsDegreesAndRadians)
{
  EXPECT_NEAR(degToRad(180.0), pi, 1e-12);
  EXPECT_NEAR(radToDeg(degToRad(37.5)), 37.5, 1e-12);
}

TEST(Frames, OpticalAxesMapToBodyAxesWithoutTilt)
{
  expectVecNear(cameraToBody({0.0, 0.0, 1.0}, {}), {1.0, 0.0, 0.0});
  expectVecNear(cameraToBody({1.0, 0.0, 0.0}, {}), {0.0, 1.0, 0.0});
  expectVecNear(cameraToBody({0.0, 1.0, 0.0}, {}), {0.0, 0.0, 1.0});
}

TEST(Frames, TiltUpLiftsOpticalAxis)
{
  // Setup
  CameraMount mount{.tiltUpDeg = 30.0};

  // Run
  Vec3 body = cameraToBody({0.0, 0.0, 1.0}, mount);

  // Assert: z is down, so up is negative z
  expectVecNear(body, {std::cos(degToRad(30.0)), 0.0, -std::sin(degToRad(30.0))});
}

TEST(Frames, CameraBodyRoundTrip)
{
  CameraMount mount{.tiltUpDeg = 17.0};
  Vec3 ray = normalized({0.3, -0.2, 0.9});
  expectVecNear(bodyToCamera(cameraToBody(ray, mount), mount), ray);
}

TEST(Frames, PitchUpLiftsForwardAxisInLevelFrame)
{
  expectVecNear(bodyToLevel({1.0, 0.0, 0.0}, 0.0, degToRad(10.0)), {std::cos(degToRad(10.0)), 0.0, -std::sin(degToRad(10.0))});
}

TEST(Frames, RollRightLowersRightAxisInLevelFrame)
{
  expectVecNear(bodyToLevel({0.0, 1.0, 0.0}, degToRad(20.0), 0.0), {0.0, std::cos(degToRad(20.0)), std::sin(degToRad(20.0))});
}

TEST(Frames, YawEastPointsForwardAxisEast)
{
  expectVecNear(bodyToNed({1.0, 0.0, 0.0}, 0.0, 0.0, degToRad(90.0)), {0.0, 1.0, 0.0});
}

TEST(Frames, NedBodyRoundTrip)
{
  Vec3 ned{3.0, -2.0, 1.0};
  expectVecNear(bodyToNed(nedToBody(ned, 0.1, -0.2, 2.5), 0.1, -0.2, 2.5), ned);
}

TEST(Frames, AngleBetweenPerpendicularVectorsIsRightAngle)
{
  EXPECT_NEAR(angleBetween({1.0, 0.0, 0.0}, {0.0, 2.0, 0.0}), pi / 2.0, 1e-12);
}
