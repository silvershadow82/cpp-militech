#pragma once

#include <optional>
#include <string>

#include <opencv2/videoio.hpp>

#include "Types.h"
#include "interfaces/IFrameSource.h"

namespace follow::providers {

struct PiCameraConfig {
  int captureWidth{1640};   // IMX219 full-FOV binned mode
  int captureHeight{1232};  // cropping to a smaller mode would narrow the field of view
  int trackWidth{640};
  int trackHeight{480};
  int fps{20};
  bool hflip{false};  // set when the airframe mounts the camera mirrored left/right
  bool vflip{false};  // set when the airframe mounts the camera upside-down
};

// The GStreamer pipeline cv::VideoCapture opens. Pure: testable without a camera or GStreamer.
std::string piCameraPipeline(const PiCameraConfig& config);

// Consecutive failed grabs tolerated before the camera counts as dead: ceil(1.5 * fps), i.e. 1.5 s
// of frames. That is past the first half of lost_timeout_ms, so the core has long since gone Lost
// and been commanding zero velocity before the app gives up on the camera. At least 1, so a
// nonsense frame rate still ends the source instead of retrying for ever.
// Pure: testable without a camera or GStreamer.
int piCameraFailureBudget(int fps);

// The Pi camera behind libcamerasrc. Throws std::runtime_error if the pipeline cannot be opened,
// which is what happens when OpenCV was built without GStreamer or no camera is attached.
class PiCameraSource final : public interfaces::IFrameSource {
public:
  explicit PiCameraSource(const PiCameraConfig& config);

  // Stamped with the steady clock when the frame was grabbed. A failed grab returns nullopt and is
  // treated as a transient miss -- appsink drop=true max-buffers=1 drops buffers by design -- until
  // piCameraFailureBudget(fps) of them arrive in a row.
  std::optional<interfaces::Frame> read() override;

  bool ended() const override { return this->consecutiveFailures >= this->failureBudget; }

private:
  cv::VideoCapture capture;
  int failureBudget;
  int consecutiveFailures{0};
};

}  // namespace follow::providers
