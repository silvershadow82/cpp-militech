#include "providers/CameraTrackerSource.h"

#include <cmath>
#include <utility>

namespace follow::providers {

models::BBox toBBox(const cv::Rect& rect)
{
  return models::BBox{.x = static_cast<double>(rect.x),
                      .y = static_cast<double>(rect.y),
                      .w = static_cast<double>(rect.width),
                      .h = static_cast<double>(rect.height)};
}

cv::Rect toRect(const models::BBox& box)
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

CameraTrackerSource::CameraTrackerSource(interfaces::IFrameSource& frames,
                                         std::unique_ptr<interfaces::ITracker> tracker,
                                         const CameraTrackerConfig& config,
                                         util::Channels& channels)
  : frames(frames)
  , tracker(std::move(tracker))
  , config(config)
  , channels(channels)
{
}

bool CameraTrackerSource::iterate()
{
  std::optional<interfaces::Frame> next = this->frames.read();
  if (!next) {
    // A dropped frame is not the end of the stream, and ending the app on one would leave the FC
    // holding the last velocity target for its whole guided timeout. Keep the previous frame for
    // the overlay, publish nothing, and let the staleness rule in TargetEstimator do the rest.
    if (this->frames.ended()) {
      return false;  // The terminal read is an ending, not a miss: it must not inflate missedFrames().
    }
    ++this->missed;
    return true;
  }
  this->frame = std::move(next);
  for (const control::TrackerRequest& request : this->channels.trackerRequests.drain()) {
    this->handle(request, *this->frame);
  }
  if (!this->isLocked) {
    return true;
  }

  std::optional<cv::Rect> box = this->tracker->update(this->frame->image);
  this->lastUpdateOk = box.has_value();
  // OpenCV's KCF and CSRT only report success or failure, so confidence is 1 or 0 (spec §Vision adapter).
  // min_confidence can therefore never reject a box: the estimator's own rules -- staleness, border
  // margin, attitude freshness and the area jump -- are the only defence against the tracker drifting
  // onto background, which is why Reacquire below must not throw a healthy lock away.
  models::TargetObservation observation{.tFrame = this->frame->t, .box = {}, .ok = box.has_value(), .confidence = box ? 1.0 : 0.0};
  if (box) {
    observation.box = toBBox(*box);
  }
  this->channels.observation.write(observation, this->frame->t);

  if (this->lastUpdateOk) {
    this->reacquiring = false;
  }
  else if (this->reacquiring) {
    // Core::step emits a TrackerRequest only on a state edge, so exactly one Reacquire arrives per
    // entry into Lost. The retry is the adapter's: keep re-seeding on the latched hint once per
    // reacquirePeriod for as long as the tracker keeps failing (spec: "reacquire retried every
    // 500 ms on the hint expanded x1.5").
    this->reseed(*this->frame);
  }
  return true;
}

void CameraTrackerSource::handle(const control::TrackerRequest& request, const interfaces::Frame& frame)
{
  cv::Rect hint = toRect(request.hint) & cv::Rect(0, 0, frame.image.cols, frame.image.rows);
  switch (request.kind) {
    case control::TrackerRequestKind::LockCenter:
      if (hint.area() > 0) {
        this->tracker->init(frame.image, hint);
        this->isLocked = true;
        this->lastUpdateOk = true;  // a freshly seeded tracker counts as healthy until it says otherwise
        this->reacquiring = false;
        this->lastReacquire.reset();
      }
      break;
    case control::TrackerRequestKind::Reacquire: {
      if (!this->isLocked) {
        break;
      }
      // The core also enters Lost for estimator-side reasons -- staleness, border margin, area jump --
      // while the tracker is still locked on the target. The spec's "the adapter must keep tracking in
      // that case and treat the request as a no-op" is read here as "do nothing at all", not merely
      // "do not unlock": re-seeding a healthy tracker on a box grown x1.5 around where the target was
      // hands KCF a patch that is mostly background, and a background lock is reported ok with
      // confidence 1.0 for ever after, with nothing downstream able to detect it.
      if (this->lastUpdateOk) {
        break;
      }
      // The tracker really has failed. Latch the hint and re-seed now; iterate() retries it once per
      // reacquirePeriod until the tracker recovers, since no second Reacquire will ever arrive.
      this->reacquireHint = request.hint;
      this->reacquiring = true;
      this->reseed(frame);
      break;
    }
    case control::TrackerRequestKind::Unlock:
      this->isLocked = false;
      this->reacquiring = false;
      break;
    case control::TrackerRequestKind::None:
      break;
  }
}

void CameraTrackerSource::reseed(const interfaces::Frame& frame)
{
  if (this->lastReacquire && frame.t - *this->lastReacquire < this->config.reacquirePeriod) {
    return;
  }
  cv::Rect grown = expandBox(toRect(this->reacquireHint), this->config.reacquireExpand, frame.image.size());
  if (grown.area() > 0) {
    this->tracker->init(frame.image, grown);
    this->lastReacquire = frame.t;
  }
}

}  // namespace follow::providers
