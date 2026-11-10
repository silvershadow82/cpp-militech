#pragma once

#include <optional>

#include <opencv2/core.hpp>

#include "Types.h"
#include "interfaces/IFrameSource.h"

namespace follow::providers {

// A bright target crossing a textured background, for tests and for exercising the adapter on a
// machine with no camera. Deterministic: frames are stamped 50 ms apart from `start`.
class SyntheticFrameSource final : public interfaces::IFrameSource {
public:
  SyntheticFrameSource(int width, int height, models::TimePoint start);

  std::optional<interfaces::Frame> read() override;

  // Where the target was drawn in the most recent frame, for tests to score against.
  cv::Rect groundTruth() const { return this->truth; }

private:
  int width;
  int height;
  models::TimePoint next;
  int index{0};
  cv::Rect truth{};
  cv::Mat background{};  // rendered once: on a Pi 4B, redrawing it per frame costs real budget
};

}  // namespace follow::providers
