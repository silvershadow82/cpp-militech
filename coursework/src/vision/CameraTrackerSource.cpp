#include "follow/vision/CameraTrackerSource.h"

#include <cmath>
#include <utility>

namespace follow::vision {

core::BBox toBBox(const cv::Rect& rect)
{
  return core::BBox{.x = static_cast<double>(rect.x),
                    .y = static_cast<double>(rect.y),
                    .w = static_cast<double>(rect.width),
                    .h = static_cast<double>(rect.height)};
}

cv::Rect toRect(const core::BBox& box)
{
  return cv::Rect(static_cast<int>(std::lround(box.x)),
                  static_cast<int>(std::lround(box.y)),
                  static_cast<int>(std::lround(box.w)),
                  static_cast<int>(std::lround(box.h)));
}

cv::Rect expandBox(const cv::Rect& box, double factor, const cv::Size& bounds)
{
  int w = static_cast<int>(std::lround(box.width * factor));
  int h = static_cast<int>(std::lround(box.height * factor));
  cv::Rect grown(box.x - (w - box.width) / 2, box.y - (h - box.height) / 2, w, h);
  return grown & cv::Rect(0, 0, bounds.width, bounds.height);
}

CameraTrackerSource::CameraTrackerSource(IFrameSource& frames,
                                         std::unique_ptr<ITracker> tracker,
                                         const CameraTrackerConfig& config,
                                         runtime::Channels& channels)
  : frames(frames)
  , tracker(std::move(tracker))
  , config(config)
  , channels(channels)
{
}

bool CameraTrackerSource::iterate()
{
  this->frame = this->frames.read();
  if (!this->frame) {
    return false;
  }
  for (const core::TrackerRequest& request : this->channels.trackerRequests.drain()) {
    this->handle(request, *this->frame);
  }
  if (!this->isLocked) {
    return true;
  }

  std::optional<cv::Rect> box = this->tracker->update(this->frame->image);
  // OpenCV's KCF and CSRT only report success or failure, so confidence is 1 or 0 (spec §Vision adapter).
  core::TargetObservation observation{.tFrame = this->frame->t, .box = {}, .ok = box.has_value(), .confidence = box ? 1.0 : 0.0};
  if (box) {
    observation.box = toBBox(*box);
  }
  this->channels.observation.write(observation, this->frame->t);
  return true;
}

void CameraTrackerSource::handle(const core::TrackerRequest& request, const Frame& frame)
{
  cv::Rect hint = toRect(request.hint) & cv::Rect(0, 0, frame.image.cols, frame.image.rows);
  switch (request.kind) {
    case core::TrackerRequestKind::LockCenter:
      if (hint.area() > 0) {
        this->tracker->init(frame.image, hint);
        this->isLocked = true;
        this->lastReacquire.reset();
      }
      break;
    case core::TrackerRequestKind::Reacquire: {
      // The core also enters Lost for estimator-side reasons (area jump, border, staleness), so a
      // Reacquire can arrive while the tracker still follows. It re-initializes on the hint, which is
      // the last box the estimator accepted, at most once per reacquirePeriod.
      if (!this->isLocked) {
        break;
      }
      if (this->lastReacquire && frame.t - *this->lastReacquire < this->config.reacquirePeriod) {
        break;
      }
      cv::Rect grown = expandBox(toRect(request.hint), this->config.reacquireExpand, frame.image.size());
      if (grown.area() > 0) {
        this->tracker->init(frame.image, grown);
        this->lastReacquire = frame.t;
      }
      break;
    }
    case core::TrackerRequestKind::Unlock:
      this->isLocked = false;
      break;
    case core::TrackerRequestKind::None:
      break;
  }
}

}  // namespace follow::vision
