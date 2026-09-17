#include "follow/vision/PiCameraSource.h"

#include <stdexcept>
#include <string>

namespace follow::vision {

std::string piCameraPipeline(const PiCameraConfig& config)
{
  // libcamerasrc captures at the sensor's full-FOV mode; videoscale brings it down to the tracking
  // size on the GPU-adjacent path rather than in our own hot loop. drop=true with max-buffers=1
  // keeps the newest frame only: falling behind real time is worse than skipping a frame, because
  // the estimator rejects observations older than its staleness window.
  //
  // libcamerasrc pads each NV21 row up to a 32-byte boundary (confirmed on a Pi 4B/imx219: at
  // captureWidth=1640 the real per-row stride is 1664, not 1640) but attaches no GstVideoMeta to
  // say so. Every downstream element that assumes a tightly-packed row -- videoconvert, videoscale,
  // and even OpenCV's own appsink ingestion -- then silently misreads every row after the first,
  // which showed up on hardware as a diagonal shear with colour banding, worse toward the bottom of
  // the frame. rawvideoparse is told the real, padded NV21 layout explicitly (plane-strides /
  // plane-offsets) so videoconvert reads the correct bytes; use-sink-caps=false makes it trust those
  // properties over the (wrong) caps libcamerasrc negotiated.
  const int strideBytes = ((config.captureWidth + 31) / 32) * 32;
  const int uvOffset = strideBytes * config.captureHeight;
  return "libcamerasrc ! video/x-raw,width=" + std::to_string(config.captureWidth) + ",height=" + std::to_string(config.captureHeight) +
         ",framerate=" + std::to_string(config.fps) +
         "/1,format=NV21 ! rawvideoparse use-sink-caps=false format=nv21 width=" + std::to_string(config.captureWidth) +
         " height=" + std::to_string(config.captureHeight) + " framerate=" + std::to_string(config.fps) + "/1 plane-strides=\"<" +
         std::to_string(strideBytes) + "," + std::to_string(strideBytes) + ">\" plane-offsets=\"<0," + std::to_string(uvOffset) +
         ">\" ! videoconvert ! videoscale ! video/x-raw,width=" + std::to_string(config.trackWidth) +
         ",height=" + std::to_string(config.trackHeight) + ",format=BGR ! appsink drop=true max-buffers=1";
}

PiCameraSource::PiCameraSource(const PiCameraConfig& config)
  : capture(piCameraPipeline(config), cv::CAP_GSTREAMER)
{
  if (!this->capture.isOpened()) {
    throw std::runtime_error("cannot open the Pi camera pipeline; check that OpenCV has GStreamer support and a camera is attached");
  }
}

std::optional<Frame> PiCameraSource::read()
{
  cv::Mat image;
  if (!this->capture.read(image) || image.empty()) {
    return std::nullopt;
  }
  // Stamp after the grab returns: this is as close to the capture instant as this API allows.
  return Frame{.image = image, .t = core::Clock::now()};
}

}  // namespace follow::vision
