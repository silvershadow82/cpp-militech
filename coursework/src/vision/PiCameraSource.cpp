#include "follow/vision/PiCameraSource.h"

#include <stdexcept>
#include <string>

namespace follow::vision {

std::string piCameraPipeline(const PiCameraConfig& config)
{
  // libcamerasrc captures at the sensor's full-FOV mode; videoscale brings it down to the tracking
  // size on the GPU-adjacent path rather than in our own hot loop. drop=true with max-buffers=1
  // keeps the newest frame only: falling behind real time is worse than skipping a frame, because
  // the estimator rejects observations older than its staleness window.
  //
  // On this stack -- Raspberry Pi OS bullseye, libcamera v0.0.5+83-bde9b04f (built 17-07-2023),
  // GStreamer 1.18.4, OpenCV 4.5.1, Pi 4B/imx219 -- libcamerasrc pads each NV21 row up to a
  // 32-byte boundary (confirmed on hardware: at captureWidth=1640 the real per-row stride is
  // 1664, not 1640) but attaches no GstVideoMeta to say so. Every downstream element that
  // assumes a tightly-packed row -- videoconvert, videoscale, and even OpenCV's own appsink
  // ingestion -- then silently misreads every row after the first, which showed up on hardware
  // as a diagonal shear with colour banding, worse toward the bottom of the frame.
  //
  // Because libcamerasrc never attaches GstVideoMeta on this stack, there is no signal to detect
  // at runtime and branch on -- the override below is therefore applied unconditionally, not
  // behind a check for "is this buffer padded". rawvideoparse is told the real, padded NV21
  // layout explicitly (plane-strides / plane-offsets) so videoconvert reads the correct bytes;
  // use-sink-caps=false makes it trust those properties over the (wrong) caps libcamerasrc
  // negotiated.
  //
  // This override is only correct for a semi-planar 4:2:0 layout: NV21 is one luma plane
  // followed by one interleaved chroma plane, so plane-strides/plane-offsets each carry exactly
  // two entries (Y, then UV) and the UV plane's stride is assumed equal to the Y plane's. A
  // 3-plane format (e.g. I420) or a packed format (e.g. YUY2) would need a different property
  // shape here, not just different numbers.
  //
  // If this ever runs against different hardware or a newer libcamera, re-check all of these
  // before trusting the numbers below, since any of them changing makes this override wrong
  // (either it does nothing useful, or it corrupts a buffer that was already correct):
  //   - whether libcamerasrc now attaches GstVideoMeta (if so, this workaround becomes
  //     redundant, and if the real stride ever differs from what we compute, actively wrong);
  //   - whether the row alignment is still 32 bytes (a different ISP/allocator could pad
  //     differently, e.g. 16 or 64);
  //   - whether NV21 is still the format libcamerasrc negotiates for this capture size (a
  //     different default, e.g. YUYV or a packed format, would violate the semi-planar
  //     assumption above).
  // A caps mismatch from any of these fails loudly with GStreamer's "not-negotiated" error
  // rather than silently corrupting frames, which is why this is documented instead of guarded
  // with a runtime check we have no second libcamera build to validate against.
  const int strideBytes = ((config.captureWidth + 31) / 32) * 32;
  const int uvOffset = strideBytes * config.captureHeight;
  return "libcamerasrc ! video/x-raw,width=" + std::to_string(config.captureWidth) + ",height=" + std::to_string(config.captureHeight) +
         ",framerate=" + std::to_string(config.fps) +
         "/1,format=NV21 ! rawvideoparse use-sink-caps=false format=nv21 width=" + std::to_string(config.captureWidth) +
         " height=" + std::to_string(config.captureHeight) + " framerate=" + std::to_string(config.fps) + "/1 plane-strides=\"<" +
         std::to_string(strideBytes) + "," + std::to_string(strideBytes) + ">\" plane-offsets=\"<0," + std::to_string(uvOffset) +
         ">\" ! videoconvert ! videoscale ! video/x-raw,width=" + std::to_string(config.trackWidth) +
         ",height=" + std::to_string(config.trackHeight) + ",format=BGR ! appsink drop=true max-buffers=1";
}

PiCameraSource::PiCameraSource(const PiCameraConfig& config)
  : capture(piCameraPipeline(config), cv::CAP_GSTREAMER)
{
  if (!this->capture.isOpened()) {
    throw std::runtime_error("cannot open the Pi camera pipeline; check that OpenCV has GStreamer support and a camera is attached");
  }
}

std::optional<Frame> PiCameraSource::read()
{
  cv::Mat image;
  if (!this->capture.read(image) || image.empty()) {
    return std::nullopt;
  }
  // Stamp after the grab returns: this is as close to the capture instant as this API allows.
  return Frame{.image = image, .t = core::Clock::now()};
}

}  // namespace follow::vision
