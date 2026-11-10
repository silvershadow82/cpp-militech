#include "providers/VideoFileSource.h"

#include <opencv2/imgproc.hpp>
#include <chrono>
#include <stdexcept>

namespace follow::providers {

VideoFileSource::VideoFileSource(const std::filesystem::path &path, const cv::Size &trackSize, double fps, models::TimePoint start)
  : capture(path.string())
  , trackSize(trackSize)
  , period(std::chrono::duration_cast<models::Clock::duration>(std::chrono::duration<double>(1.0 / fps)))
  , next(start)
{
  if (!this->capture.isOpened()) {
    throw std::runtime_error("cannot open video " + path.string());
  }
}

std::optional<interfaces::Frame> VideoFileSource::read()
{
  cv::Mat raw;
  if (!this->capture.read(raw) || raw.empty()) {
    this->atEnd = true;
    return std::nullopt;
  }
  interfaces::Frame frame{.image = {}, .t = this->next};
  if (raw.size() == this->trackSize) {
    frame.image = raw;
  }
  else {
    cv::resize(raw, frame.image, this->trackSize, 0.0, 0.0, cv::INTER_AREA);
  }
  this->next += this->period;
  return frame;
}

}  // namespace follow::providers
