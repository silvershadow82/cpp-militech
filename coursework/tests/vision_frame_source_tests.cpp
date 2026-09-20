#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "providers/FrameSource.h"

TEST(SyntheticFrameSource, StampsFramesOnAFixedCadence)
{
  follow::models::TimePoint start = follow::models::Clock::now();
  follow::providers::SyntheticFrameSource source(640, 480, start);

  std::optional<follow::providers::Frame> first = source.read();
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(first->t, start);
  EXPECT_EQ(first->image.cols, 640);
  EXPECT_EQ(first->image.rows, 480);

  std::optional<follow::providers::Frame> second = source.read();
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(second->t - first->t, std::chrono::milliseconds{50});
}

TEST(SyntheticFrameSource, MovesTheTargetBetweenFrames)
{
  follow::providers::SyntheticFrameSource source(640, 480, follow::models::Clock::now());
  source.read();
  cv::Rect first = source.groundTruth();
  source.read();
  cv::Rect second = source.groundTruth();
  EXPECT_GT(second.x, first.x);
}

TEST(VideoFileSource, ReadsEveryFrameAndResizesToTheTrackingSize)
{
  std::filesystem::path clip = std::filesystem::temp_directory_path() / "follow_frame_source_test.avi";
  {
    cv::VideoWriter writer(clip.string(), cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 20.0, cv::Size(1280, 960));
    ASSERT_TRUE(writer.isOpened());
    for (int i = 0; i < 10; ++i) {
      cv::Mat frame(960, 1280, CV_8UC3, cv::Scalar(30, 30, 30));
      cv::rectangle(frame, cv::Rect(200 + 5 * i, 300, 60, 180), cv::Scalar(240, 240, 240), -1);
      writer.write(frame);
    }
  }

  follow::providers::VideoFileSource source(clip, cv::Size(640, 480), 20.0, follow::models::Clock::now());
  int count = 0;
  std::optional<follow::providers::Frame> frame;
  while ((frame = source.read())) {
    EXPECT_EQ(frame->image.cols, 640);
    EXPECT_EQ(frame->image.rows, 480);
    ++count;
  }
  EXPECT_EQ(count, 10);
  std::filesystem::remove(clip);
}

TEST(VideoFileSource, MissingFileThrows)
{
  EXPECT_THROW(follow::providers::VideoFileSource("/nonexistent/clip.avi", cv::Size(640, 480), 20.0, follow::models::Clock::now()),
               std::runtime_error);
}

TEST(SyntheticFrameSource, NeverEnds)
{
  // The synthetic source is the app's stand-in for a live camera: it has no EOF, so a caller may
  // keep reading for as long as it likes.
  follow::providers::SyntheticFrameSource source(640, 480, follow::models::Clock::now());
  source.read();
  EXPECT_FALSE(source.ended());
}

TEST(VideoFileSource, ReportsExhaustionOnlyAfterTheLastFrame)
{
  std::filesystem::path clip = std::filesystem::temp_directory_path() / "follow_frame_source_end_test.avi";
  {
    cv::VideoWriter writer(clip.string(), cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 20.0, cv::Size(640, 480));
    ASSERT_TRUE(writer.isOpened());
    for (int i = 0; i < 3; ++i) {
      writer.write(cv::Mat(480, 640, CV_8UC3, cv::Scalar(30, 30, 30)));
    }
  }

  follow::providers::VideoFileSource source(clip, cv::Size(640, 480), 20.0, follow::models::Clock::now());
  EXPECT_FALSE(source.ended());
  for (int i = 0; i < 3; ++i) {
    ASSERT_TRUE(source.read().has_value());
    EXPECT_FALSE(source.ended());
  }
  EXPECT_FALSE(source.read().has_value());
  EXPECT_TRUE(source.ended());  // a clip that ran out really is exhausted, unlike a dropped buffer
  std::filesystem::remove(clip);
}
