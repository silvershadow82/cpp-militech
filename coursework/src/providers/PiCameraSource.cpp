#include "providers/PiCameraSource.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace follow::providers {

std::string piCameraPipeline(const PiCameraConfig &config)
{
  const int strideBytes = ((config.captureWidth + 31) / 32) * 32;
  const int uvOffset = strideBytes * config.captureHeight;

  std::string pipeline = "libcamerasrc ! video/x-raw,width=" + std::to_string(config.captureWidth) +
                         ",height=" + std::to_string(config.captureHeight) + ",framerate=" + std::to_string(config.fps) +
                         "/1,format=NV21 ! rawvideoparse use-sink-caps=false format=nv21 width=" + std::to_string(config.captureWidth) +
                         " height=" + std::to_string(config.captureHeight) + " framerate=" + std::to_string(config.fps) +
                         "/1 plane-strides=\"<" + std::to_string(strideBytes) + "," + std::to_string(strideBytes) +
                         ">\" plane-offsets=\"<0," + std::to_string(uvOffset) +
                         ">\" ! videoconvert ! videoscale ! video/x-raw,width=" + std::to_string(config.trackWidth) +
                         ",height=" + std::to_string(config.trackHeight) + ",format=BGR";

  if (config.hflip && config.vflip) {
    pipeline += " ! videoflip method=rotate-180";
  }
  else if (config.hflip) {
    pipeline += " ! videoflip method=horizontal-flip";
  }
  else if (config.vflip) {
    pipeline += " ! videoflip method=vertical-flip";
  }

  pipeline += " ! appsink drop=true max-buffers=1";
  return pipeline;
}

int piCameraFailureBudget(int fps)
{
  return std::max(1, static_cast<int>(std::ceil(1.5 * fps)));
}

PiCameraSource::PiCameraSource(const PiCameraConfig &config)
  : capture(piCameraPipeline(config), cv::CAP_GSTREAMER)
  , failureBudget(piCameraFailureBudget(config.fps))
{
  if (!this->capture.isOpened()) {
    throw std::runtime_error("cannot open the Pi camera pipeline; check that OpenCV has GStreamer support and a camera is attached");
  }
}

std::optional<interfaces::Frame> PiCameraSource::read()
{
  cv::Mat image;
  if (!this->capture.read(image) || image.empty()) {
    ++this->consecutiveFailures;
    return std::nullopt;
  }
  this->consecutiveFailures = 0;

  return interfaces::Frame{.image = image, .t = models::Clock::now()};
}

}  // namespace follow::providers
