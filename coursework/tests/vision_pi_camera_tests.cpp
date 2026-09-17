#include <gtest/gtest.h>

#include <string>

#include "follow/vision/PiCameraSource.h"

TEST(PiCameraPipeline, DescribesTheCaptureAndTrackingSizes)
{
  follow::vision::PiCameraConfig config{};
  config.captureWidth = 1640;
  config.captureHeight = 1232;
  config.trackWidth = 640;
  config.trackHeight = 480;
  config.fps = 20;

  std::string pipeline = follow::vision::piCameraPipeline(config);

  EXPECT_NE(pipeline.find("libcamerasrc"), std::string::npos);
  EXPECT_NE(pipeline.find("width=1640"), std::string::npos);
  EXPECT_NE(pipeline.find("height=1232"), std::string::npos);
  EXPECT_NE(pipeline.find("width=640"), std::string::npos);
  EXPECT_NE(pipeline.find("height=480"), std::string::npos);
  EXPECT_NE(pipeline.find("framerate=20/1"), std::string::npos);
  // appsink is what cv::VideoCapture reads from; without it the pipeline yields no frames.
  EXPECT_NE(pipeline.find("appsink"), std::string::npos);
  // BGR is what cv::Mat expects; a pipeline ending in any other format gives garbled colour.
  EXPECT_NE(pipeline.find("format=BGR"), std::string::npos);
}

TEST(PiCameraPipeline, DropsLateFramesRatherThanQueueingThem)
{
  std::string pipeline = follow::vision::piCameraPipeline(follow::vision::PiCameraConfig{});
  // A queueing appsink makes the tracker fall progressively further behind real time; the
  // estimator's staleness rule would then invalidate every observation.
  EXPECT_NE(pipeline.find("drop=true"), std::string::npos);
  EXPECT_NE(pipeline.find("max-buffers=1"), std::string::npos);
}
