#pragma once

#include <optional>
#include <string>

#include <opencv2/videoio.hpp>

#include "follow/core/Types.h"
#include "follow/vision/FrameSource.h"

namespace follow::vision {

struct PiCameraConfig {
  int captureWidth{1640};   // IMX219 full-FOV binned mode
  int captureHeight{1232};  // cropping to a smaller mode would narrow the field of view
  int trackWidth{640};
  int trackHeight{480};
  int fps{20};
};

// The GStreamer pipeline cv::VideoCapture opens. Pure: testable without a camera or GStreamer.
std::string piCameraPipeline(const PiCameraConfig& config);

// The Pi camera behind libcamerasrc. Throws std::runtime_error if the pipeline cannot be opened,
// which is what happens when OpenCV was built without GStreamer or no camera is attached.
class PiCameraSource final : public IFrameSource {
public:
  explicit PiCameraSource(const PiCameraConfig& config);

  // Stamped with the steady clock when the frame was grabbed.
  std::optional<Frame> read() override;

private:
  cv::VideoCapture capture;
};

}  // namespace follow::vision
