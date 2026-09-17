#pragma once

#include <chrono>
#include <memory>
#include <optional>

#include <opencv2/core.hpp>

#include "follow/core/Core.h"
#include "follow/core/Types.h"
#include "follow/runtime/Channels.h"
#include "follow/vision/FrameSource.h"
#include "follow/vision/Tracker.h"

namespace follow::vision {

struct CameraTrackerConfig {
  std::chrono::milliseconds reacquirePeriod{500};  // Reacquire re-initializes at most this often
  double reacquireExpand{1.5};                     // on the last good box grown by this factor
};

core::BBox toBBox(const cv::Rect& rect);
cv::Rect toRect(const core::BBox& box);
// `box` grown by `factor` around its center, clipped to an image of `bounds`.
cv::Rect expandBox(const cv::Rect& box, double factor, const cv::Size& bounds);

// The vision thread of follow_app --hw: reads frames, carries out the core's tracker requests and
// publishes one TargetObservation per frame while locked. Like SimVision, it only does I/O: every
// decision about what the observation means belongs to TargetEstimator.
class CameraTrackerSource {
public:
  CameraTrackerSource(IFrameSource& frames,
                      std::unique_ptr<ITracker> tracker,
                      const CameraTrackerConfig& config,
                      runtime::Channels& channels);

  // Reads one frame, applies pending requests to it, and publishes an observation if locked.
  // Returns false when the frame source is exhausted or failed to deliver a frame.
  bool iterate();

  // The most recent frame, for the overlay. Frames are kept even while unlocked.
  const std::optional<Frame>& lastFrame() const { return this->frame; }
  bool locked() const { return this->isLocked; }

private:
  void handle(const core::TrackerRequest& request, const Frame& frame);

  IFrameSource& frames;
  std::unique_ptr<ITracker> tracker;
  CameraTrackerConfig config;
  runtime::Channels& channels;
  std::optional<Frame> frame{};
  bool isLocked{false};
  std::optional<core::TimePoint> lastReacquire{};
};

}  // namespace follow::vision
