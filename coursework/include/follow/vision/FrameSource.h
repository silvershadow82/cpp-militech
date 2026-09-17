#pragma once

#include <filesystem>
#include <optional>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include "follow/core/Types.h"

namespace follow::vision {

// One captured frame at the tracking resolution, stamped when it was captured. The estimator
// compensates bearing for the yaw change since this time, so the stamp must be the capture time,
// not the time tracking finished.
struct Frame {
  cv::Mat image{};
  core::TimePoint t{};
};

class IFrameSource {
public:
  virtual ~IFrameSource() = default;

  // The next frame, or nullopt when the source is exhausted or a capture failed.
  virtual std::optional<Frame> read() = 0;
};

// A bright target crossing a textured background, for tests and for exercising the adapter on a
// machine with no camera. Deterministic: frames are stamped 50 ms apart from `start`.
class SyntheticFrameSource final : public IFrameSource {
public:
  SyntheticFrameSource(int width, int height, core::TimePoint start);

  std::optional<Frame> read() override;

  // Where the target was drawn in the most recent frame, for tests to score against.
  cv::Rect groundTruth() const { return this->truth; }

private:
  int width;
  int height;
  core::TimePoint next;
  int index{0};
  cv::Rect truth{};
};

// A recorded clip, resized to the tracking size. Frames are stamped from `fps`, so a clip plays
// back with the timing it was recorded at. Throws std::runtime_error if the file cannot be opened.
class VideoFileSource final : public IFrameSource {
public:
  VideoFileSource(const std::filesystem::path& path, const cv::Size& trackSize, double fps, core::TimePoint start);

  std::optional<Frame> read() override;

private:
  cv::VideoCapture capture;
  cv::Size trackSize;
  core::Clock::duration period;
  core::TimePoint next;
};

}  // namespace follow::vision
