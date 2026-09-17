#pragma once

#include <cstddef>
#include <stdexcept>
#include <string>

#include <opencv2/core.hpp>

#include "follow/core/Core.h"

namespace follow::vision {

// The overlay label for each state (spec §State machine): READY, LOCK, FOLLOW, LOST, HOLD, NO FC.
const char* overlayText(core::State state);

// Draws the lock box, the target box (if any) and the state label onto `image`, in place.
void drawOverlay(cv::Mat& image, const core::OverlayInfo& info);

// Pure, OS-independent arithmetic for FramebufferWriter. Kept out of the `#ifdef __linux__` guard so
// it compiles and is unit-testable on any host, including one with no /dev/fb*.
namespace detail {

// A framebuffer's reported geometry: two independent ioctls (FBIOGET_VSCREENINFO,
// FBIOGET_FSCREENINFO) are free to disagree, and a panned or rotated console is free to differ from
// both.
struct FramebufferGeometry {
  int width{0};                  // panel width, pixels
  int height{0};                 // panel height, pixels
  int bytesPerPixel{0};          // 2 (BGR565) or 4 (BGRA)
  int lineLength{0};             // bytes per scanline (fix_screeninfo.line_length)
  int xoffset{0};                // pan offset, pixels (var_screeninfo.xoffset)
  int yoffset{0};                // pan offset, lines (var_screeninfo.yoffset)
  std::size_t mappingLength{0};  // bytes actually mapped (fix_screeninfo.smem_len)
};

// The byte offset of pixel (xoffset, yoffset) within a mapping of `mappingLength` bytes. Throws
// std::runtime_error, naming the mismatch, when the unpanned line is not wide enough for the panel
// (`width * bytesPerPixel > lineLength`), the *panned* line overruns it
// (`(xoffset + width) * bytesPerPixel > lineLength` -- a panned console can fit the width at xoffset
// 0 and still spill into the next scanline once panned), or writing `height` panned lines would run
// past `mappingLength` -- i.e. whenever it would be unsafe to write through the mapping at all.
// Checking this once, here, replaces finding out via SIGSEGV/SIGBUS through the MAP_SHARED mapping
// later (an unchecked horizontal overrun stays inside the mapping's *bounds*, so it corrupts the
// next scanline rather than segfaulting -- a diagonally sheared overlay with no error, not a crash).
std::size_t framebufferWriteOffset(const FramebufferGeometry& geometry);

// The destination rectangle for a `srcSize` image, scaled to fit inside a `panelWidth`x`panelHeight`
// panel without changing its aspect ratio, and centred (letterboxed/pillarboxed rather than
// stretched). `srcSize`, `panelWidth` and `panelHeight` must all be positive.
cv::Rect letterboxRect(const cv::Size& srcSize, int panelWidth, int panelHeight);

// The red/green/blue bit offsets a framebuffer reports (var_screeninfo.{red,green,blue}.offset).
struct ChannelOffsets {
  unsigned redOffset{0};
  unsigned greenOffset{0};
  unsigned blueOffset{0};
};

// Throws std::runtime_error, naming the observed and expected layouts, unless `observed` matches
// what cv::COLOR_BGR2BGR565 (16bpp) or cv::COLOR_BGR2BGRA (32bpp) actually produce in memory. A
// framebuffer reporting a different layout (RGB565, ARGB, ...) would otherwise render with red and
// blue swapped with nothing to report it.
void requireExpectedChannelLayout(int bitsPerPixel, const ChannelOffsets& observed);

}  // namespace detail

// A Linux framebuffer (/dev/fb0) mapped into memory. The composite output shows whatever is written
// here. Throws std::runtime_error if the device cannot be opened, queried or mapped, which is always
// the case on a machine without Linux framebuffers.
class FramebufferWriter {
public:
  explicit FramebufferWriter(const std::string& device);
  ~FramebufferWriter();
  FramebufferWriter(const FramebufferWriter&) = delete;
  FramebufferWriter& operator=(const FramebufferWriter&) = delete;

  // Scales a BGR image to the framebuffer resolution and writes it in the framebuffer's pixel format.
  void write(const cv::Mat& bgr);

private:
  int fd{-1};
  unsigned char* pixels{nullptr};
  std::size_t length{0};
  int width{0};
  int height{0};
  int bitsPerPixel{0};
  int lineLength{0};
  // Byte offset of (xoffset, yoffset) in the mapping; see detail::framebufferWriteOffset. Computed
  // once in the constructor -- a console that pans or changes mode afterwards is not re-read.
  std::size_t writeOffset{0};
  cv::Mat canvas{};     // panel-sized BGR buffer; recleared by write() whenever the letterbox rect changes
  cv::Rect lastRect{};  // the letterbox rect canvas was last cleared for; see write()
};

}  // namespace follow::vision
