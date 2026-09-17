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
};

}  // namespace follow::vision
