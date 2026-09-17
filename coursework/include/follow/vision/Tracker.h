#pragma once

#include <memory>
#include <optional>
#include <string>

#include <opencv2/core.hpp>

namespace follow::vision {

// A single-target tracker. One implementation per OpenCV tracker; the adapter never sees OpenCV's
// tracker types directly, so a tracker with a real confidence score can replace these later.
class ITracker {
public:
  virtual ~ITracker() = default;

  // Start tracking `box` in `frame`. Called again on every re-lock.
  virtual void init(const cv::Mat& frame, const cv::Rect& box) = 0;

  // The new box, or nullopt when the tracker reports failure.
  virtual std::optional<cv::Rect> update(const cv::Mat& frame) = 0;
};

// "kcf" or "csrt"; throws std::invalid_argument for any other name.
std::unique_ptr<ITracker> makeTracker(const std::string& name);

}  // namespace follow::vision
