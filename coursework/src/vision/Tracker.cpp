#include "vision/Tracker.h"

// OpenCV moved KCF and CSRT between modules: the contrib "tracking" module carries them on both
// 4.6 (Pi OS, libopencv-contrib-dev) and 5.0 (Homebrew). Fall back to the main video module, which
// also declares them from 4.5.1 on, so a build without contrib still compiles.
#if __has_include(<opencv2/tracking.hpp>)
#include <opencv2/tracking.hpp>
#else
#include <opencv2/video/tracking.hpp>
#endif

#include <stdexcept>

namespace follow::vision {

namespace {

// Both OpenCV trackers share this shape: create() on init, update() returning success plus a box.
class OpenCvTracker final : public ITracker {
public:
  using Factory = cv::Ptr<cv::Tracker> (*)();

  explicit OpenCvTracker(Factory factory)
    : factory(factory)
  {
  }

  void init(const cv::Mat& frame, const cv::Rect& box) override
  {
    this->tracker = this->factory();
    this->tracker->init(frame, box);
  }

  std::optional<cv::Rect> update(const cv::Mat& frame) override
  {
    if (!this->tracker) {
      return std::nullopt;
    }
    cv::Rect box;
    if (!this->tracker->update(frame, box)) {
      return std::nullopt;
    }
    return box;
  }

private:
  Factory factory;
  cv::Ptr<cv::Tracker> tracker{};
};

}  // namespace

std::unique_ptr<ITracker> makeTracker(const std::string& name)
{
  if (name == "kcf") {
    return std::make_unique<OpenCvTracker>([]() -> cv::Ptr<cv::Tracker> { return cv::TrackerKCF::create(); });
  }
  if (name == "csrt") {
    return std::make_unique<OpenCvTracker>([]() -> cv::Ptr<cv::Tracker> { return cv::TrackerCSRT::create(); });
  }
  throw std::invalid_argument("vision.tracker: expected \"kcf\" or \"csrt\", got \"" + name + "\"");
}

}  // namespace follow::vision
