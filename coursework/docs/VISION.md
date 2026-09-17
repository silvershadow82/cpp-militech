# Vision build, hardware mode and tools

The vision pipeline runs on the Raspberry Pi with OpenCV to track a target from the camera feed, providing measurements to the follow controller over shared slots.

## Building with OpenCV

The vision library and tools require OpenCV and GStreamer. Configure the build with `FOLLOW_WITH_OPENCV=ON`:

```bash
cmake -S coursework -B build/vision -DFOLLOW_WITH_OPENCV=ON
cmake --build build/vision -j8
```

### Installation

On Raspberry Pi OS, install the libraries:

```bash
sudo apt install libopencv-dev libopencv-contrib-dev gstreamer1.0-libcamera
```

The contrib package provides `opencv2/tracking.hpp` (KCF and CSRT trackers). The GStreamer package provides libcamerasrc and its libcamera integration; OpenCV also needs GStreamer support compiled into its build (cv::CAP_GSTREAMER).

On macOS with Homebrew, OpenCV is installed but GStreamer is not available:

```bash
brew install opencv
```

`--hw` mode cannot open the Pi camera on macOS, but `--sim` and all vision tools work.

## Why FOLLOW_WITH_OPENCV defaults to OFF

The option defaults to `OFF` because:

- The devcontainer build (gcc-13) has no OpenCV installed.
- The aarch64 cross-compile toolchain has no sysroot; `find_package(OpenCV)` would fail.

CI builds (default configuration and aarch64 debug cross-build) must remain free of OpenCV. Turn it on only when building natively on the Pi or a machine with OpenCV installed.

## follow_app --hw

Run the hardware mode with:

```bash
./build/vision/follow_app --hw [--config FILE] [--link SPEC] [--log FILE]
```

It opens the Pi camera, tracks the target, and streams MAVLink commands to the flight controller. Press Ctrl-C to stop.

### Configuration

The vision configuration is in `follow.json` (or overridden via `--config FILE`). All keys have defaults; missing values are left unchanged.

| Key | Default | Description |
| --- | --- | --- |
| `vision.capture` | [1640, 1232] | Camera capture resolution (pixels). Calibration is performed at this resolution. |
| `vision.track` | [640, 480] | Tracking resolution (pixels). Frames are scaled to this size; all tracker coordinates are in this space. |
| `vision.tracker` | "kcf" | Tracker algorithm: "kcf" or "csrt". |
| `vision.lock_box_frac` | 0.20 | Side of the lock box as a fraction of the image height. |
| `vision.min_confidence` | 0.3 | Minimum tracker confidence (0–1) to continue following. Below this, the target is considered lost. |
| `vision.fps` | 20 | Camera frame rate (Hz) requested from libcamerasrc. |
| `vision.framebuffer` | "/dev/fb0" | Linux framebuffer for the overlay. Leave empty to disable the overlay. |
| `vision.overlay_fps` | 15 | Overlay redraw rate (Hz). |
| `vision.reacquire_period_ms` | 500 | Time between reacquisition attempts when lost (milliseconds). |
| `vision.reacquire_expand` | 1.5 | Expansion factor (> 1.0) of the lock box during reacquisition. |

### Overlay behavior

When `/dev/fb0` exists, frames with tracking overlays (lock box and target box) are written at `overlay_fps` Hz. If the framebuffer is missing or cannot be opened, a message is printed and tracking continues without the overlay.

### Camera pipeline: libcamerasrc row-stride workaround

On first hardware bring-up (Pi 4B, imx219, Raspberry Pi OS bullseye, libcamera v0.0.5+83-bde9b04f built 17-07-2023, GStreamer 1.18.4, OpenCV 4.5.1), `follow_app --hw` produced overlay frames with a visible defect: diagonal shearing and green/magenta colour banding, worse toward the bottom of the frame, at the full `vision.capture` resolution (1640x1232). The overlay draw itself (lock box, state text) was correct; only the underlying camera image was corrupted.

Root cause: on this stack, `libcamerasrc` pads each NV21 row up to a 32-byte boundary (1640 pads to 1664 bytes) but attaches no `GstVideoMeta` describing that padding. Downstream elements that assume a tightly-packed row — `videoconvert`, `videoscale`, and OpenCV's own `appsink` ingestion — all silently misread every row after the first. 640x480 (the tracking resolution) is unaffected because 640 is already a multiple of 32.

The fix, in `PiCameraSource::piCameraPipeline`, inserts a `rawvideoparse` stage right after `libcamerasrc` that states the real, padded NV21 layout explicitly (`plane-strides`/`plane-offsets`, computed from `captureWidth`/`captureHeight` rounded up to the next 32-byte boundary). This is applied unconditionally — there is nothing to detect at runtime, since the missing `GstVideoMeta` is exactly the absence of a signal to branch on — and it assumes a 2-plane semi-planar 4:2:0 layout (NV21: one luma plane, one interleaved chroma plane).

**This is specific to the libcamera/GStreamer stack above.** Before relying on it on different hardware or a newer libcamera build, re-check on that hardware:

- whether `libcamerasrc` now attaches `GstVideoMeta` (if so, the override is redundant at best, and actively wrong if the real stride it reports differs from what this code computes);
- whether the row alignment is still 32 bytes (a different ISP/allocator could pad to a different boundary, e.g. 16 or 64);
- whether NV21 is still the format `libcamerasrc` negotiates at the requested capture size (a different default, e.g. a packed or 3-plane format, would violate the semi-planar assumption the `rawvideoparse` properties depend on).

Any of these changing makes the override wrong and it must be revisited. A caps mismatch from a wrong assumption fails loudly with GStreamer's "not-negotiated" error rather than silently corrupting frames, which is why this is handled by documenting the assumption instead of adding untested runtime detection.

### MAVLink link

The default link is `uart:/dev/serial0:921600` (the Pi's UART to the flight controller). Override via `--link`:

```bash
./build/vision/follow_app --hw --link udp:14560:127.0.0.1:14560
```

### Run log

Tracking decisions, estimator state, and control commands are logged to CSV format (default `follow_run.csv`). Override with `--log`:

```bash
./build/vision/follow_app --hw --log my_flight.csv
```

## Tracker bench

Benchmark the tracker on a recorded video:

1. On the drone, record a clip with the target centered in the first frame:

```bash
rpicam-vid --width 640 --height 480 --framerate 20 --codec mjpeg -o clip.mjpeg
```

2. Transfer the clip and run the bench tool:

```bash
./build/vision/follow_tracker_bench clip.mjpeg --tracker kcf --out annotated.avi
```

This locks on the centered box in the first frame and tracks to the end. It does not re-lock after a loss; it reports total frames, frames per second (tracker only), and the number of losses (transitions from tracking into failure).

### Acceptance criteria

The tracker must maintain **at least 15 fps** at 640×480 on a Pi 4B. Slower hardware or lossy patterns (occlusion, blur, rapid turns) will drop the frame rate.

### Options

```bash
follow_tracker_bench VIDEO [--tracker kcf|csrt] [--out FILE] [--lock-box-frac F] [--track WxH]
```

- `--tracker`: "kcf" (default) or "csrt". KCF is faster; CSRT is more robust.
- `--out`: Write an annotated AVI with MJPG codec. Omit to skip.
- `--lock-box-frac`: Lock box side as a fraction of image height (default 0.20).
- `--track`: Tracking resolution (default 640x480).

## Calibration

Fisheye calibration corrects the camera distortion and computes intrinsics for the tracker. Run once during bring-up.

1. Print a checkerboard (e.g., 9×6 inner corners, 0.025 m squares) on a flat surface (paper or cardboard).

2. Capture 15–25 images at the full capture resolution, angling the board to cover the corners of the fisheye image:

```bash
rpicam-still --width 1640 --height 1232 -r --timelapse 100 captures/img_%05d.jpg
```

3. Run the calibration tool:

```bash
./build/vision/follow_calibrate_fisheye captures/ --board 9x6 --square 0.025 --out config/camera_imx219_160.json
```

### Calibration file format

The output is a camera JSON with intrinsics at the capture resolution:

```json
{
  "model": "fisheye",
  "width": 1640,
  "height": 1232,
  "fx": 1234.56,
  "fy": 1234.56,
  "cx": 820.0,
  "cy": 616.0,
  "k1": -0.01,
  "k2": 0.002,
  "k3": -0.0001,
  "k4": 0.0001,
  "tilt_deg": 0.0
}
```

Keys: `model`, `width`, `height`, `fx`, `fy`, `cx`, `cy`, `k1`, `k2`, `k3`, `k4`, `tilt_deg`. The tilt angle is used by the TargetEstimator's bearing geometry; an incorrect tilt skews bearing and distance estimates.

### Pass/fail

The tool computes the reprojection error (RMS) over all calibration views. It **passes** when RMS < 0.5 px (exits 0) and **fails** when RMS ≥ 0.5 px (exits 3). This threshold is the bring-up stage 0 gate. Poor image quality, incorrect board size, or too few views with all corners visible will cause failure.

### Options

```bash
follow_calibrate_fisheye IMAGE_DIR --board WxH --square M [--tilt DEG] [--out FILE]
```

- `IMAGE_DIR`: Directory of checkerboard images (.png or .jpg), captured at `vision.capture` resolution.
- `--board`: Checkerboard inner corner count (e.g., 9x6).
- `--square`: Square side in metres (e.g., 0.025 for 2.5 cm).
- `--tilt`: Camera tilt up from body forward, in degrees (default 0). Used by the TargetEstimator's bearing geometry (`cameraToBody` and `bodyToCamera` frame conversions); an incorrect tilt skews bearing and distance estimates.
- `--out`: Output camera JSON file (default `camera_imx219_160.json`).

## OpenCV 4 vs 5 include guards

The codebase handles OpenCV 4.x and 5.0 API differences with conditional includes:

### Trackers (Tracker.cpp)

```cpp
#if __has_include(<opencv2/tracking.hpp>)
#include <opencv2/tracking.hpp>
#else
#include <opencv2/video/tracking.hpp>
#endif
```

The contrib `tracking` module carries KCF and CSRT trackers on both OpenCV 4.6 (Pi OS with libopencv-contrib-dev) and 5.0 (Homebrew). The guard checks for contrib's presence (4.5.1 and later). If contrib is not installed, the fallback is `video/tracking.hpp`, which carries the trackers on OpenCV 4.5.1 and later, so a build without contrib still compiles.

### Chessboard detection (Calibration.cpp)

```cpp
#if __has_include(<opencv2/objdetect.hpp>)
#include <opencv2/objdetect.hpp>
#endif
```

OpenCV 5.0 moved chessboard detection to `objdetect.hpp`. On OpenCV 4.x, the function is also in `calib3d.hpp` (automatically included). The conditional include prevents redeclaration errors on OpenCV 5.0.

### Fisheye calibration (Calibration.cpp)

```cpp
#include <opencv2/calib3d.hpp>
```

Fisheye calibration (`cv::fisheye::calibrate` and `cv::fisheye::CALIB_*` constants) live in `calib3d.hpp` on both OpenCV 4 and 5. No guard is needed.

These three guards ensure the vision code compiles against both OpenCV versions without modification.
