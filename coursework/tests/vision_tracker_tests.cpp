#include <gtest/gtest.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <stdexcept>

#include "follow/vision/Tracker.h"

namespace {

// A bright target on a textured background, moving right and slightly up, as the tracker sees it.
cv::Mat renderFrame(int index, cv::Rect& truth)
{
  cv::Mat frame(480, 640, CV_8UC3, cv::Scalar(40, 45, 50));
  for (int k = 0; k < 60; ++k) {
    cv::circle(frame, cv::Point((k * 97) % 640, (k * 53) % 480), 3, cv::Scalar(90, 90, 90), -1);
  }
  truth = cv::Rect(160 + 3 * index, 200 - index / 4, 38, 118);
  cv::rectangle(frame, truth, cv::Scalar(220, 210, 200), -1);
  cv::circle(frame, cv::Point(truth.x + 19, truth.y + 12), 12, cv::Scalar(180, 160, 150), -1);
  return frame;
}

double centerError(const cv::Rect& box, const cv::Rect& truth)
{
  double dx = (box.x + box.width / 2.0) - (truth.x + truth.width / 2.0);
  double dy = (box.y + box.height / 2.0) - (truth.y + truth.height / 2.0);
  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace

TEST(VisionTracker, KcfFollowsAMovingTarget)
{
  cv::Rect truth;
  cv::Mat first = renderFrame(0, truth);
  auto tracker = follow::vision::makeTracker("kcf");
  tracker->init(first, truth);

  double worst = 0.0;
  for (int i = 1; i < 40; ++i) {
    cv::Mat frame = renderFrame(i, truth);
    std::optional<cv::Rect> box = tracker->update(frame);
    ASSERT_TRUE(box.has_value()) << "tracker reported failure on frame " << i;
    worst = std::max(worst, centerError(*box, truth));
  }
  EXPECT_LT(worst, 25.0);
}

TEST(VisionTracker, CsrtFollowsAMovingTarget)
{
  cv::Rect truth;
  cv::Mat first = renderFrame(0, truth);
  auto tracker = follow::vision::makeTracker("csrt");
  tracker->init(first, truth);

  for (int i = 1; i < 20; ++i) {
    cv::Mat frame = renderFrame(i, truth);
    ASSERT_TRUE(tracker->update(frame).has_value()) << "tracker reported failure on frame " << i;
  }
}

TEST(VisionTracker, UnknownNameThrows)
{
  EXPECT_THROW(follow::vision::makeTracker("mosse"), std::invalid_argument);
}

TEST(VisionTracker, UpdateBeforeInitReportsFailure)
{
  auto tracker = follow::vision::makeTracker("kcf");
  cv::Rect truth;
  cv::Mat frame = renderFrame(0, truth);
  EXPECT_FALSE(tracker->update(frame).has_value());
}
