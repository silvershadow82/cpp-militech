#include "providers/FrameSource.h"

#include <chrono>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace follow::providers {

SyntheticFrameSource::SyntheticFrameSource(int width, int height, models::TimePoint start)
  : width(width)
  , height(height)
  , next(start)
{
}

std::optional<interfaces::Frame> SyntheticFrameSource::read()
{
  if (this->background.empty()) {
    this->background = cv::Mat(this->height, this->width, CV_8UC3, cv::Scalar(40, 45, 50));
    // Static texture, so the tracker has a background to discriminate the target against. It never
    // changes, so render it once and clone it: re-drawing 60 circles into a fresh 900 KiB Mat every
    // frame is a real share of the 50 ms frame budget on the Pi.
    for (int k = 0; k < 60; ++k) {
      cv::circle(this->background, cv::Point((k * 97) % this->width, (k * 53) % this->height), 3, cv::Scalar(90, 90, 90), -1);
    }
  }
  cv::Mat image = this->background.clone();
  this->truth = cv::Rect(this->width / 2 - 19 + 3 * this->index, this->height / 2 - 59 - this->index / 4, 38, 118);
  cv::rectangle(image, this->truth, cv::Scalar(220, 210, 200), -1);
  cv::circle(image, cv::Point(this->truth.x + 19, this->truth.y + 12), 12, cv::Scalar(180, 160, 150), -1);

  interfaces::Frame frame{.image = image, .t = this->next};
  ++this->index;
  this->next += std::chrono::milliseconds{50};
  return frame;
}

VideoFileSource::VideoFileSource(const std::filesystem::path& path, const cv::Size& trackSize, double fps, models::TimePoint start)
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
