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
  return "libcamerasrc ! video/x-raw,width=" + std::to_string(config.captureWidth) + ",height=" + std::to_string(config.captureHeight) +
         ",framerate=" + std::to_string(config.fps) +
         "/1 ! videoconvert ! videoscale ! video/x-raw,width=" + std::to_string(config.trackWidth) +
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
