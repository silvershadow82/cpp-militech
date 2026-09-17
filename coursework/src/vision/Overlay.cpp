#include "follow/vision/Overlay.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <sstream>

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

namespace detail {

std::size_t framebufferWriteOffset(const FramebufferGeometry& geometry)
{
  if (geometry.bytesPerPixel != 2 && geometry.bytesPerPixel != 4) {
    throw std::runtime_error("unsupported " + std::to_string(geometry.bytesPerPixel * 8) + " bits per pixel");
  }
  if (geometry.xoffset < 0 || geometry.yoffset < 0) {
    throw std::runtime_error("negative pan offset (" + std::to_string(geometry.xoffset) + "," + std::to_string(geometry.yoffset) + ")");
  }
  if (static_cast<std::size_t>(geometry.width) * static_cast<std::size_t>(geometry.bytesPerPixel) >
      static_cast<std::size_t>(geometry.lineLength)) {
    throw std::runtime_error("line_length " + std::to_string(geometry.lineLength) + " is too short for a " +
                             std::to_string(geometry.width) + "px line at " + std::to_string(geometry.bytesPerPixel) + " bytes/pixel");
  }
  std::size_t base =
    static_cast<std::size_t>(geometry.yoffset) * geometry.lineLength + static_cast<std::size_t>(geometry.xoffset) * geometry.bytesPerPixel;
  std::size_t needed = base + static_cast<std::size_t>(geometry.height) * geometry.lineLength;
  if (needed > geometry.mappingLength) {
    throw std::runtime_error("a " + std::to_string(geometry.width) + "x" + std::to_string(geometry.height) + " panel panned to (" +
                             std::to_string(geometry.xoffset) + "," + std::to_string(geometry.yoffset) + ") needs " +
                             std::to_string(needed) + " bytes but only " + std::to_string(geometry.mappingLength) + " are mapped");
  }
  return base;
}

cv::Rect letterboxRect(const cv::Size& srcSize, int panelWidth, int panelHeight)
{
  if (srcSize.width <= 0 || srcSize.height <= 0 || panelWidth <= 0 || panelHeight <= 0) {
    throw std::runtime_error("letterboxRect: sizes must be positive");
  }
  double scale = std::min(static_cast<double>(panelWidth) / srcSize.width, static_cast<double>(panelHeight) / srcSize.height);
  int w = std::max(1, static_cast<int>(std::lround(srcSize.width * scale)));
  int h = std::max(1, static_cast<int>(std::lround(srcSize.height * scale)));
  return cv::Rect((panelWidth - w) / 2, (panelHeight - h) / 2, w, h);
}

void requireExpectedChannelLayout(int bitsPerPixel, const ChannelOffsets& observed)
{
  // What cv::COLOR_BGR2BGR565 (16bpp) and cv::COLOR_BGR2BGRA (32bpp) actually produce in memory.
  ChannelOffsets expected = bitsPerPixel == 32 ? ChannelOffsets{.redOffset = 16, .greenOffset = 8, .blueOffset = 0}
                                               : ChannelOffsets{.redOffset = 11, .greenOffset = 5, .blueOffset = 0};
  if (observed.redOffset != expected.redOffset || observed.greenOffset != expected.greenOffset ||
      observed.blueOffset != expected.blueOffset) {
    std::ostringstream message;
    message << bitsPerPixel << "bpp channel layout red@" << observed.redOffset << " green@" << observed.greenOffset << " blue@"
            << observed.blueOffset << " does not match the expected red@" << expected.redOffset << " green@" << expected.greenOffset
            << " blue@" << expected.blueOffset;
    throw std::runtime_error(message.str());
  }
}

}  // namespace detail

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
  try {
    this->writeOffset = detail::framebufferWriteOffset(detail::FramebufferGeometry{.width = this->width,
                                                                                   .height = this->height,
                                                                                   .bytesPerPixel = this->bitsPerPixel / 8,
                                                                                   .lineLength = this->lineLength,
                                                                                   .xoffset = static_cast<int>(var.xoffset),
                                                                                   .yoffset = static_cast<int>(var.yoffset),
                                                                                   .mappingLength = this->length});
    detail::requireExpectedChannelLayout(
      this->bitsPerPixel,
      detail::ChannelOffsets{.redOffset = var.red.offset, .greenOffset = var.green.offset, .blueOffset = var.blue.offset});
  }
  catch (const std::runtime_error& e) {
    ::close(this->fd);
    throw std::runtime_error("framebuffer " + device + ": " + e.what());
  }
  void* mapped = ::mmap(nullptr, this->length, PROT_READ | PROT_WRITE, MAP_SHARED, this->fd, 0);
  if (mapped == MAP_FAILED) {
    ::close(this->fd);
    throw std::runtime_error("cannot map framebuffer " + device);
  }
  this->pixels = static_cast<unsigned char*>(mapped);
  // Cleared once: write() always letterboxes into the same rect for a fixed-size source, so the
  // margins stay black without needing to be re-cleared every frame.
  this->canvas = cv::Mat::zeros(this->height, this->width, CV_8UC3);
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
  // Letterbox rather than stretch: a 4:3 frame on a 16:9 (or any non-4:3) panel keeps its aspect
  // ratio, so a square lock box still renders square. The rect is deterministic for a fixed-size
  // source, so the canvas's margins -- cleared once in the constructor -- stay cleared.
  cv::Rect rect = detail::letterboxRect(bgr.size(), this->width, this->height);
  cv::Mat scaled;
  cv::resize(bgr, scaled, rect.size(), 0.0, 0.0, cv::INTER_LINEAR);
  scaled.copyTo(this->canvas(rect));
  cv::Mat converted;
  if (this->bitsPerPixel == 32) {
    cv::cvtColor(this->canvas, converted, cv::COLOR_BGR2BGRA);
  }
  else {
    cv::cvtColor(this->canvas, converted, cv::COLOR_BGR2BGR565);
  }
  const int bytesPerPixel = this->bitsPerPixel / 8;
  unsigned char* base = this->pixels + this->writeOffset;
  for (int row = 0; row < this->height; ++row) {
    std::memcpy(
      base + static_cast<std::size_t>(row) * this->lineLength, converted.ptr(row), static_cast<std::size_t>(this->width) * bytesPerPixel);
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
