#pragma once

#include <optional>

#include <opencv2/core.hpp>

#include "Types.h"

namespace follow::interfaces {

// One captured frame at the tracking resolution, stamped when it was captured. The estimator
// compensates bearing for the yaw change since this time, so the stamp must be the capture time,
// not the time tracking finished.
//
// Frame lives here rather than in models/ on purpose: it holds a cv::Mat, and models/ is
// OpenCV-free by rule. It belongs to the interface it is passed across.
struct Frame {
  cv::Mat image{};
  models::TimePoint t{};
};

class IFrameSource {
public:
  virtual ~IFrameSource() = default;

  // The next frame, or nullopt when no frame was available this tick. A nullopt on its own is a
  // transient miss -- a dropped buffer, a renegotiation, an ISP hiccup -- which the caller must
  // keep flying through; only ended() says the source can never deliver another frame.
  virtual std::optional<Frame> read() = 0;

  // True once the source is exhausted and read() will never succeed again. A source with no end
  // (a live camera that is still healthy, the synthetic source) never reports true.
  virtual bool ended() const { return false; }
};

}  // namespace follow::interfaces
