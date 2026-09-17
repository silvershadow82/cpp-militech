#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "follow/vision/FrameSource.h"

TEST(SyntheticFrameSource, StampsFramesOnAFixedCadence)
{
  follow::core::TimePoint start = follow::core::Clock::now();
  follow::vision::SyntheticFrameSource source(640, 480, start);

  std::optional<follow::vision::Frame> first = source.read();
  ASSERT_TRUE(first.has_value());
  EXPECT_EQ(first->t, start);
  EXPECT_EQ(first->image.cols, 640);
  EXPECT_EQ(first->image.rows, 480);

  std::optional<follow::vision::Frame> second = source.read();
  ASSERT_TRUE(second.has_value());
  EXPECT_EQ(second->t - first->t, std::chrono::milliseconds{50});
}

TEST(SyntheticFrameSource, MovesTheTargetBetweenFrames)
{
  follow::vision::SyntheticFrameSource source(640, 480, follow::core::Clock::now());
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

  follow::vision::VideoFileSource source(clip, cv::Size(640, 480), 20.0, follow::core::Clock::now());
  int count = 0;
  std::optional<follow::vision::Frame> frame;
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
  EXPECT_THROW(follow::vision::VideoFileSource("/nonexistent/clip.avi", cv::Size(640, 480), 20.0, follow::core::Clock::now()),
               std::runtime_error);
}
