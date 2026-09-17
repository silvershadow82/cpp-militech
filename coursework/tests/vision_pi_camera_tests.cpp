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

TEST(PiCameraPipeline, CorrectsForLibcamerasrcRowStridePadding)
{
  // Confirmed on a Pi 4B/imx219: libcamerasrc emits NV21 buffers whose rows the vc4 ISP pads to a
  // 32-byte boundary, but it attaches no GstVideoMeta describing that padding. Every downstream
  // element that assumes a tightly-packed row (stride == captureWidth) then misreads the buffer --
  // videoconvert, videoscale and even OpenCV's own appsink ingestion -- which showed up on hardware
  // as a diagonal shear with colour banding, worse toward the bottom of the frame. rawvideoparse is
  // told the real, padded layout explicitly (format/width/height plus plane-strides/plane-offsets)
  // so videoconvert reads the correct bytes. 1640 is not a multiple of 32, so it pads to 1664.
  follow::vision::PiCameraConfig config{};
  config.captureWidth = 1640;
  config.captureHeight = 1232;

  std::string pipeline = follow::vision::piCameraPipeline(config);

  EXPECT_NE(pipeline.find("format=NV21"), std::string::npos);
  EXPECT_NE(pipeline.find("rawvideoparse"), std::string::npos);
  EXPECT_NE(pipeline.find("format=nv21"), std::string::npos);
  // 1640 rounded up to the next 32-byte boundary is 1664; the Y and UV planes share that row stride.
  EXPECT_NE(pipeline.find("plane-strides=\"<1664,1664>\""), std::string::npos);
  // The UV plane starts right after the (padded) Y plane: 1664 * 1232.
  EXPECT_NE(pipeline.find("plane-offsets=\"<0,2050048>\""), std::string::npos);
}

TEST(PiCameraPipeline, StrideMatchesWidthWhenCaptureWidthIsAlready32Aligned)
{
  // 640 is already a multiple of 32, so no padding correction is needed: stride == width.
  follow::vision::PiCameraConfig config{};
  config.captureWidth = 640;
  config.captureHeight = 480;

  std::string pipeline = follow::vision::piCameraPipeline(config);

  EXPECT_NE(pipeline.find("plane-strides=\"<640,640>\""), std::string::npos);
  EXPECT_NE(pipeline.find("plane-offsets=\"<0,307200>\""), std::string::npos);  // 640 * 480
}

TEST(PiCameraPipeline, RoundsAnArbitraryNonAlignedWidthUpTo32Bytes)
{
  // Neither production value (1640 nor 640): proves the rounding formula itself, not just the
  // two constants it happens to be called with today. 100 is not a multiple of 32; the next one
  // up is 128 (4 * 32).
  follow::vision::PiCameraConfig config{};
  config.captureWidth = 100;
  config.captureHeight = 50;

  std::string pipeline = follow::vision::piCameraPipeline(config);

  EXPECT_NE(pipeline.find("plane-strides=\"<128,128>\""), std::string::npos);
  EXPECT_NE(pipeline.find("plane-offsets=\"<0,6400>\""), std::string::npos);  // 128 * 50
}
