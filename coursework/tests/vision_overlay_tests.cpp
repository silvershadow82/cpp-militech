#include <gtest/gtest.h>

#include <cstring>

#include <opencv2/core.hpp>

#include "follow/vision/Overlay.h"

using follow::core::BBox;
using follow::core::OverlayInfo;
using follow::core::State;

TEST(OverlayText, MatchesTheSpecLabels)
{
  EXPECT_STREQ(follow::vision::overlayText(State::Idle), "READY");
  EXPECT_STREQ(follow::vision::overlayText(State::Locking), "LOCK");
  EXPECT_STREQ(follow::vision::overlayText(State::Following), "FOLLOW");
  EXPECT_STREQ(follow::vision::overlayText(State::Lost), "LOST");
  EXPECT_STREQ(follow::vision::overlayText(State::Hold), "HOLD");
  EXPECT_STREQ(follow::vision::overlayText(State::NoFc), "NO FC");
}

TEST(DrawOverlay, DrawsTheLockBoxAndTheTargetBox)
{
  cv::Mat image(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  OverlayInfo info{.state = State::Following,
                   .lockBox = BBox{.x = 272.0, .y = 192.0, .w = 96.0, .h = 96.0},
                   .targetBox = BBox{.x = 100.0, .y = 100.0, .w = 40.0, .h = 120.0}};

  follow::vision::drawOverlay(image, info);

  // Lock box outline: top-left corner pixel is painted, its interior is not.
  EXPECT_NE(image.at<cv::Vec3b>(192, 272), cv::Vec3b(0, 0, 0));
  EXPECT_EQ(image.at<cv::Vec3b>(240, 320), cv::Vec3b(0, 0, 0));
  // Target box outline is painted.
  EXPECT_NE(image.at<cv::Vec3b>(100, 120), cv::Vec3b(0, 0, 0));
  // State text lands in the top-left corner.
  cv::Mat corner = image(cv::Rect(0, 0, 200, 40));
  EXPECT_GT(cv::countNonZero(corner.reshape(1)), 0);
}

TEST(DrawOverlay, WithoutATargetOnlyTheLockBoxAndTextAreDrawn)
{
  cv::Mat image(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
  OverlayInfo info{.state = State::Idle, .lockBox = BBox{.x = 272.0, .y = 192.0, .w = 96.0, .h = 96.0}, .targetBox = std::nullopt};
  follow::vision::drawOverlay(image, info);
  EXPECT_EQ(image.at<cv::Vec3b>(100, 120), cv::Vec3b(0, 0, 0));
}

TEST(FramebufferWriter, MissingDeviceThrows)
{
  EXPECT_THROW(follow::vision::FramebufferWriter("/nonexistent/fb0"), std::runtime_error);
}
