#pragma once

#include <filesystem>
#include <optional>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

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

// A recorded clip, resized to the tracking size. Frames are stamped from `fps`, so a clip plays
// back with the timing it was recorded at. Throws std::runtime_error if the file cannot be opened.
class VideoFileSource final : public interfaces::IFrameSource {
public:
  VideoFileSource(const std::filesystem::path& path, const cv::Size& trackSize, double fps, models::TimePoint start);

  std::optional<interfaces::Frame> read() override;

  // True once the clip has run out: a file really does end, unlike a live camera.
  bool ended() const override { return this->atEnd; }

private:
  cv::VideoCapture capture;
  cv::Size trackSize;
  models::Clock::duration period;
  models::TimePoint next;
  bool atEnd{false};
};

}  // namespace follow::providers
