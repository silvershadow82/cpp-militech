#include "follow/vision/Overlay.h"

#include <cstring>

#include <opencv2/imgproc.hpp>

#include "follow/vision/CameraTrackerSource.h"

#ifdef __linux__
#include <fcntl.h>
#include <linux/fb.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#endif

namespace follow::vision {

const char* overlayText(core::State state)
{
  switch (state) {
    case core::State::Idle:
      return "READY";
    case core::State::Locking:
      return "LOCK";
    case core::State::Following:
      return "FOLLOW";
    case core::State::Lost:
      return "LOST";
    case core::State::Hold:
      return "HOLD";
    case core::State::NoFc:
      return "NO FC";
  }
  return "?";
}

void drawOverlay(cv::Mat& image, const core::OverlayInfo& info)
{
  const cv::Scalar white(255, 255, 255);
  const cv::Scalar green(0, 255, 0);
  cv::rectangle(image, toRect(info.lockBox), white, 1);
  if (info.targetBox) {
    cv::rectangle(image, toRect(*info.targetBox), green, 2);
  }
  // Analog video is low resolution: a thick label with a dark outline stays readable.
  cv::putText(image, overlayText(info.state), cv::Point(12, 30), cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 0, 0), 4);
  cv::putText(image, overlayText(info.state), cv::Point(12, 30), cv::FONT_HERSHEY_SIMPLEX, 0.9, green, 2);
}

#ifdef __linux__

FramebufferWriter::FramebufferWriter(const std::string& device)
{
  this->fd = ::open(device.c_str(), O_RDWR);
  if (this->fd < 0) {
    throw std::runtime_error("cannot open framebuffer " + device);
  }
  fb_var_screeninfo var{};
  fb_fix_screeninfo fix{};
  if (::ioctl(this->fd, FBIOGET_VSCREENINFO, &var) != 0 || ::ioctl(this->fd, FBIOGET_FSCREENINFO, &fix) != 0) {
    ::close(this->fd);
    throw std::runtime_error("cannot query framebuffer " + device);
  }
  this->width = static_cast<int>(var.xres);
  this->height = static_cast<int>(var.yres);
  this->bitsPerPixel = static_cast<int>(var.bits_per_pixel);
  this->lineLength = static_cast<int>(fix.line_length);
  this->length = fix.smem_len;
  if (this->bitsPerPixel != 16 && this->bitsPerPixel != 32) {
    ::close(this->fd);
    throw std::runtime_error("framebuffer " + device + ": unsupported " + std::to_string(this->bitsPerPixel) + " bits per pixel");
  }
  void* mapped = ::mmap(nullptr, this->length, PROT_READ | PROT_WRITE, MAP_SHARED, this->fd, 0);
  if (mapped == MAP_FAILED) {
    ::close(this->fd);
    throw std::runtime_error("cannot map framebuffer " + device);
  }
  this->pixels = static_cast<unsigned char*>(mapped);
}

FramebufferWriter::~FramebufferWriter()
{
  if (this->pixels) {
    ::munmap(this->pixels, this->length);
  }
  if (this->fd >= 0) {
    ::close(this->fd);
  }
}

void FramebufferWriter::write(const cv::Mat& bgr)
{
  cv::Mat scaled;
  cv::resize(bgr, scaled, cv::Size(this->width, this->height), 0.0, 0.0, cv::INTER_LINEAR);
  cv::Mat converted;
  if (this->bitsPerPixel == 32) {
    cv::cvtColor(scaled, converted, cv::COLOR_BGR2BGRA);
  }
  else {
    cv::cvtColor(scaled, converted, cv::COLOR_BGR2BGR565);
  }
  const int bytesPerPixel = this->bitsPerPixel / 8;
  for (int row = 0; row < this->height; ++row) {
    std::memcpy(this->pixels + static_cast<std::size_t>(row) * this->lineLength,
                converted.ptr(row),
                static_cast<std::size_t>(this->width) * bytesPerPixel);
  }
}

#else

// Only Linux has /dev/fb*; elsewhere the writer cannot exist, which callers already handle.
FramebufferWriter::FramebufferWriter(const std::string& device)
{
  throw std::runtime_error("framebuffer " + device + ": framebuffers are only available on Linux");
}

FramebufferWriter::~FramebufferWriter() = default;

void FramebufferWriter::write(const cv::Mat&) {}

#endif

}  // namespace follow::vision
