#include <gtest/gtest.h>

#include <array>

#include "follow/core/Angles.h"
#include "follow/core/CameraModel.h"
#include "follow/core/Frames.h"

using namespace follow::core;

namespace {

void expectVecNear(const Vec3& actual, const Vec3& expected, double tolerance = 1e-9)
{
  EXPECT_NEAR(actual.x, expected.x, tolerance);
  EXPECT_NEAR(actual.y, expected.y, tolerance);
  EXPECT_NEAR(actual.z, expected.z, tolerance);
}

const Intrinsics kPinhole{.width = 640, .height = 480, .fx = 500.0, .fy = 500.0, .cx = 320.0, .cy = 240.0};

// Distorted intrinsics used to generate the OpenCV reference values below.
const Intrinsics kDistorted{
  .width = 640, .height = 480, .fx = 290.0, .fy = 289.0, .cx = 321.5, .cy = 238.2, .k1 = 0.05, .k2 = -0.01, .k3 = 0.002, .k4 = -0.0003};

struct Reference {
  Vec3 point;
  Pixel pixel;
};

// cv::fisheye::projectPoints(points, rvec=0, tvec=0, K, D) with OpenCV 5.0.0, K and D from kDistorted.
const std::array<Reference, 4> kOpenCvReference{{
  {{0.1, 0.2, 1.0}, {350.099043047, 295.200851315}},
  {{-0.8, 0.3, 1.0}, {125.149310206, 311.577606919}},
  {{1.5, -1.2, 1.0}, {580.401217457, 31.793236290}},
  {{-2.0, -1.5, 1.0}, {30.096803201, 20.401231358}},
}};

}  // namespace

TEST(PinholeModel, CenterPixelIsOpticalAxis)
{
  PinholeModel model(kPinhole);
  expectVecNear(model.pixelToRay({320.0, 240.0}), {0.0, 0.0, 1.0});
}

TEST(PinholeModel, PixelRayRoundTrip)
{
  PinholeModel model(kPinhole);
  std::optional<Pixel> back = model.rayToPixel(model.pixelToRay({100.0, 400.0}));
  ASSERT_TRUE(back);
  EXPECT_NEAR(back->u, 100.0, 1e-9);
  EXPECT_NEAR(back->v, 400.0, 1e-9);
}

TEST(PinholeModel, RayBehindCameraHasNoPixel)
{
  PinholeModel model(kPinhole);
  EXPECT_FALSE(model.rayToPixel({0.0, 0.0, -1.0}));
}

TEST(FisheyeKbModel, MatchesOpenCvProjectPoints)
{
  FisheyeKbModel model(kDistorted);
  for (const Reference& ref : kOpenCvReference) {
    std::optional<Pixel> pixel = model.rayToPixel(normalized(ref.point));
    ASSERT_TRUE(pixel);
    EXPECT_NEAR(pixel->u, ref.pixel.u, 1e-6);
    EXPECT_NEAR(pixel->v, ref.pixel.v, 1e-6);
  }
}

TEST(FisheyeKbModel, PixelToRayInvertsProjectionAcrossImage)
{
  FisheyeKbModel model(kDistorted);
  for (double u = 0.0; u <= 640.0; u += 40.0) {
    for (double v = 0.0; v <= 480.0; v += 40.0) {
      std::optional<Pixel> back = model.rayToPixel(model.pixelToRay({u, v}));
      ASSERT_TRUE(back);
      EXPECT_NEAR(back->u, u, 1e-6);
      EXPECT_NEAR(back->v, v, 1e-6);
    }
  }
}

TEST(FisheyeKbModel, NominalImageCornerIsHalfDiagonalFov)
{
  FisheyeKbModel model(nominalFisheye(640, 480, 160.0));
  EXPECT_NEAR(radToDeg(angleBetween(model.pixelToRay({0.0, 0.0}), {0.0, 0.0, 1.0})), 80.0, 1e-9);
}

TEST(FisheyeKbModel, NominalHorizontalHalfFovIs64Degrees)
{
  // 160 degrees across the 800 px diagonal gives 320 px * (80 deg / 400 px) = 64 deg to the side edge.
  FisheyeKbModel model(nominalFisheye(640, 480, 160.0));
  EXPECT_NEAR(radToDeg(angleBetween(model.pixelToRay({0.0, 240.0}), {0.0, 0.0, 1.0})), 64.0, 1e-9);
}

TEST(FisheyeKbModel, RayAtRightAngleHasNoPixel)
{
  FisheyeKbModel model(nominalFisheye(640, 480, 160.0));
  EXPECT_FALSE(model.rayToPixel({1.0, 0.0, 0.0}));
}

TEST(CameraModel, ContainsChecksImageBounds)
{
  PinholeModel model(kPinhole);
  EXPECT_TRUE(model.contains({0.0, 0.0}));
  EXPECT_TRUE(model.contains({640.0, 480.0}));
  EXPECT_FALSE(model.contains({-0.1, 10.0}));
  EXPECT_FALSE(model.contains({10.0, 480.1}));
}
