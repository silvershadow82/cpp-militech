#include <gtest/gtest.h>

#include <vector>

#include <nlohmann/json.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "config/ConfigJson.h"
#include "vision/Calibration.h"

namespace {

// Projects a 9x6 board through a known fisheye model from 12 poses, as a camera would see it.
std::vector<std::vector<cv::Point2f>> syntheticViews(const cv::Matx33d& K, const cv::Vec4d& D)
{
  std::vector<cv::Point3f> board;
  for (int r = 0; r < 6; ++r) {
    for (int c = 0; c < 9; ++c) {
      board.emplace_back(static_cast<float>(c * 0.025), static_cast<float>(r * 0.025), 0.0f);
    }
  }
  std::vector<std::vector<cv::Point2f>> views;
  for (int v = 0; v < 12; ++v) {
    cv::Vec3d rvec(0.05 * (v % 4) - 0.075, 0.05 * (v / 4) - 0.05, 0.01 * v);
    cv::Vec3d tvec(-0.1 + 0.01 * v, -0.06 + 0.01 * (v % 3), 0.45 + 0.02 * v);
    std::vector<cv::Point2f> points;
    cv::fisheye::projectPoints(board, points, rvec, tvec, K, D);
    views.push_back(points);
  }
  return views;
}

}  // namespace

TEST(CalibrateFisheye, RecoversAKnownModel)
{
  cv::Matx33d K(300.0, 0.0, 320.0, 0.0, 300.0, 240.0, 0.0, 0.0, 1.0);
  cv::Vec4d D(0.02, -0.005, 0.001, 0.0);

  follow::vision::CalibrationResult result =
    follow::vision::calibrateFisheye(syntheticViews(K, D), cv::Size(9, 6), 0.025, cv::Size(640, 480));

  EXPECT_LT(result.rms, 0.5);
  EXPECT_EQ(result.views, 12);
  EXPECT_NEAR(result.intrinsics.fx, 300.0, 0.5);
  EXPECT_NEAR(result.intrinsics.fy, 300.0, 0.5);
  EXPECT_NEAR(result.intrinsics.cx, 320.0, 0.5);
  EXPECT_NEAR(result.intrinsics.cy, 240.0, 0.5);
  EXPECT_NEAR(result.intrinsics.k1, 0.02, 1e-3);
  EXPECT_EQ(result.intrinsics.width, 640);
  EXPECT_EQ(result.intrinsics.height, 480);
}

TEST(CalibrateFisheye, TooFewViewsThrows)
{
  cv::Matx33d K(300.0, 0.0, 320.0, 0.0, 300.0, 240.0, 0.0, 0.0, 1.0);
  auto views = syntheticViews(K, cv::Vec4d(0, 0, 0, 0));
  views.resize(2);
  EXPECT_THROW(follow::vision::calibrateFisheye(views, cv::Size(9, 6), 0.025, cv::Size(640, 480)), std::runtime_error);
}

TEST(FindBoardCorners, FindsARenderedBoard)
{
  cv::Mat scene(480, 640, CV_8UC1, cv::Scalar(255));
  for (int r = 0; r < 7; ++r) {
    for (int c = 0; c < 10; ++c) {
      if ((r + c) % 2 == 0) {
        cv::rectangle(scene, cv::Rect(120 + c * 40, 90 + r * 40, 40, 40), cv::Scalar(0), -1);
      }
    }
  }
  auto corners = follow::vision::findBoardCorners(scene, cv::Size(9, 6));
  ASSERT_TRUE(corners.has_value());
  EXPECT_EQ(corners->size(), 54u);
}

TEST(FindBoardCorners, ReportsABlankImage)
{
  cv::Mat blank(480, 640, CV_8UC1, cv::Scalar(255));
  EXPECT_FALSE(follow::vision::findBoardCorners(blank, cv::Size(9, 6)).has_value());
}

TEST(CameraJson, IsReadBackByParseCamera)
{
  follow::vision::CalibrationResult result{};
  result.intrinsics = follow::models::Intrinsics{.width = 1640,
                                                 .height = 1232,
                                                 .fx = 600.0,
                                                 .fy = 601.0,
                                                 .cx = 820.0,
                                                 .cy = 616.0,
                                                 .k1 = 0.01,
                                                 .k2 = -0.002,
                                                 .k3 = 0.0003,
                                                 .k4 = -0.00004};
  result.rms = 0.3;
  result.views = 20;

  nlohmann::json doc = follow::vision::cameraJson(result, 12.5);
  follow::config::CameraSettings settings = follow::config::parseCamera(doc, 640, 480);

  EXPECT_EQ(settings.kind, follow::config::CameraKind::Fisheye);
  EXPECT_NEAR(settings.intrinsics.fx, 600.0 * 640.0 / 1640.0, 1e-9);
  EXPECT_NEAR(settings.intrinsics.cy, 616.0 * 480.0 / 1232.0, 1e-9);
  EXPECT_DOUBLE_EQ(settings.intrinsics.k4, -0.00004);
  EXPECT_DOUBLE_EQ(settings.mount.tiltUpDeg, 12.5);
}
