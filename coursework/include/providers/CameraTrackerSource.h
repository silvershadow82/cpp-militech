#pragma once

#include <chrono>
#include <cstdint>
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
  // Returns false only when the frame source reports it is exhausted. A frame that failed to
  // arrive is a transient miss: nothing is published and this returns true, so the estimator's
  // staleness rule takes the target to Lost and the core commands zero instead of the app exiting
  // and leaving the FC holding the last setpoint.
  bool iterate();

  // The most recent frame, for the overlay. Frames are kept even while unlocked, and a missed
  // frame leaves the previous one in place rather than blanking the pilot's overlay.
  const std::optional<Frame>& lastFrame() const { return this->frame; }
  bool locked() const { return this->isLocked; }
  // Frames the source failed to deliver since construction, for the caller to report.
  uint64_t missedFrames() const { return this->missed; }

private:
  void handle(const core::TrackerRequest& request, const Frame& frame);
  // Re-initializes the tracker on the latched hint, at most once per reacquirePeriod.
  void reseed(const Frame& frame);

  IFrameSource& frames;
  std::unique_ptr<ITracker> tracker;
  CameraTrackerConfig config;
  runtime::Channels& channels;
  std::optional<Frame> frame{};
  bool isLocked{false};
  uint64_t missed{0};
  bool lastUpdateOk{false};    // the last tracker->update() result: Reacquire is a no-op while true
  bool reacquiring{false};     // a Reacquire is outstanding and the tracker has not recovered yet
  core::BBox reacquireHint{};  // the hint that Reacquire carried, retried until the tracker recovers
  std::optional<core::TimePoint> lastReacquire{};
};

}  // namespace follow::vision
