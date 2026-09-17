#include "follow/vision/HwApp.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

#include "follow/config/ConfigJson.h"
#include "follow/mavlink/ByteLink.h"
#include "follow/runtime/Channels.h"
#include "follow/runtime/ControlLoop.h"
#include "follow/runtime/MavlinkIo.h"
#include "follow/runtime/RunLog.h"
#include "follow/vision/CameraTrackerSource.h"
#include "follow/vision/Overlay.h"
#include "follow/vision/Tracker.h"

namespace follow::vision {

namespace {

// How long shutdown waits for the I/O thread to send the zero setpoint. The thread iterates at
// least every 5 ms, so this is a bound for a link that has already failed, not a normal delay.
constexpr auto kFailsafeSendTimeout = std::chrono::milliseconds{200};

}  // namespace

void runHwApp(const HwAppOptions& options, IFrameSource& frames, const std::atomic<bool>& stop, std::ostream& out)
{
  config::AppConfig app = config::loadAppConfig(options.configPath);
  std::string linkSpec = options.link.value_or(app.mavlink.link);
  // Owned here so it outlives the Core inside ControlLoop, which keeps a reference.
  std::unique_ptr<core::CameraModel> camera = config::makeCameraModel(app.camera);
  std::unique_ptr<mavlink::ByteLink> link = mavlink::openLink(mavlink::parseLinkSpec(linkSpec));
  std::ofstream logFile(options.logPath);
  if (!logFile) {
    throw std::runtime_error("cannot write run log " + options.logPath.string());
  }

  std::mutex outMutex;
  auto print = [&out, &outMutex](const std::string& line) {
    std::lock_guard<std::mutex> lock(outMutex);
    out << line << std::endl;
  };

  std::unique_ptr<FramebufferWriter> framebuffer;
  if (!app.vision.framebuffer.empty()) {
    try {
      framebuffer = std::make_unique<FramebufferWriter>(app.vision.framebuffer);
    }
    catch (const std::runtime_error& e) {
      print(std::string("follow_app: no overlay: ") + e.what());
    }
  }

  runtime::Channels channels;
  runtime::RunLogWriter log(logFile);
  mavlink::MavlinkIds ids{.sysid = static_cast<uint8_t>(app.mavlink.sysid), .compid = static_cast<uint8_t>(app.mavlink.compid)};
  runtime::MavlinkIo io(*link, ids, channels, [&print](const std::string& text) { print("FC: " + text); });
  CameraTrackerSource source(frames,
                             makeTracker(app.vision.tracker),
                             CameraTrackerConfig{.reacquirePeriod = std::chrono::milliseconds{app.vision.reacquirePeriodMs},
                                                 .reacquireExpand = app.vision.reacquireExpand},
                             channels);
  runtime::ControlLoop control(app.core, *camera, app.camera.mount, channels, &log, core::Clock::now());

  print("follow_app --hw: tracker " + app.vision.tracker + ", link " + linkSpec + ", log " + options.logPath.string());
  // Two stop flags, not one: the I/O thread outlives the other two so the fail-safe setpoint below
  // can still be put on the wire after the control loop has stopped writing setpoints.
  std::atomic<bool> threadsStop{false};
  std::atomic<bool> ioStop{false};
  std::atomic<bool> framesEnded{false};
  const auto overlayPeriod = std::chrono::duration_cast<core::Clock::duration>(std::chrono::duration<double>(1.0 / app.vision.overlayFps));
  const auto framePeriod = std::chrono::duration_cast<core::Clock::duration>(std::chrono::duration<double>(1.0 / app.vision.fps));

  auto visionLoop = [&] {
    core::TimePoint nextOverlay = core::Clock::now();
    core::TimePoint nextFrame = core::Clock::now();
    uint64_t reportedMisses = 0;
    while (!threadsStop) {
      if (!source.iterate()) {
        framesEnded = true;
        return;
      }
      if (source.missedFrames() != reportedMisses) {
        reportedMisses = source.missedFrames();
        print("follow_app: camera missed a frame (" + std::to_string(reportedMisses) + " so far)");
      }
      core::TimePoint now = core::Clock::now();
      if (framebuffer && now >= nextOverlay && source.lastFrame()) {
        cv::Mat image = source.lastFrame()->image.clone();
        if (auto overlay = channels.overlay.read()) {
          drawOverlay(image, overlay->value);
        }
        framebuffer->write(image);
        nextOverlay = now + overlayPeriod;
      }
      // The Pi camera blocks until the next frame; a file or synthetic source does not, so pace it.
      nextFrame = std::max(nextFrame + framePeriod, now);
      std::this_thread::sleep_until(nextFrame);
    }
  };

  // An exception escaping a thread function calls std::terminate(): the process dies instantly, no
  // destructor runs, no zero setpoint is sent and the run log is truncated at the last flush. The
  // vision thread makes that reachable because it calls into OpenCV on every frame -- tracker
  // update, overlay drawing, the framebuffer's resize and colour conversion -- and OpenCV reports
  // every error by throwing. Record the first failure, stop the others, and let runHwApp rethrow it
  // below, after the fail-safe setpoint has gone out.
  std::mutex failureMutex;
  std::optional<std::string> failure;
  auto guarded = [&failureMutex, &failure, &threadsStop](const char* name, auto body) {
    return [&failureMutex, &failure, &threadsStop, name, body] {
      try {
        body();
      }
      catch (const std::exception& e) {
        {
          std::lock_guard<std::mutex> lock(failureMutex);
          if (!failure) {
            failure = std::string(name) + " thread failed: " + e.what();
          }
        }
        threadsStop = true;
      }
    };
  };

  std::thread ioThread;
  std::thread visionThread;
  std::thread controlThread;

  // Every exit path -- a clean stop, a camera that really ended, or a throw -- goes through here.
  // ArduPilot holds the last velocity target until its guided timeout (3 s), so an app that simply
  // stops leaves the vehicle flying at whatever it was last commanded. Command zero and wait for
  // the I/O thread to put it on the wire before stopping it. Nothing has touched the capture object
  // at this point: on this libcamerasrc build an end-of-stream can hang, so cv::VideoCapture's
  // release() (in the caller, after this returns) must never come first.
  auto shutdown = [&] {
    threadsStop = true;
    if (controlThread.joinable()) {
      controlThread.join();
    }
    if (visionThread.joinable()) {
      visionThread.join();
    }
    if (ioThread.joinable()) {
      // The control loop has stopped, so this is the last write to the slot.
      channels.setpoint.write(core::VelocityCmd{}, core::Clock::now());
      std::optional<runtime::Stamped<core::VelocityCmd>> zero = channels.setpoint.read();
      core::TimePoint deadline = core::Clock::now() + kFailsafeSendTimeout;
      while (zero && io.sentSetpointSequence() < zero->sequence && core::Clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds{1});
      }
      ioStop = true;
      ioThread.join();
    }
  };

  try {
    ioThread = std::thread(guarded("mavlink", [&] { io.run(ioStop); }));
    visionThread = std::thread(guarded("vision", visionLoop));
    controlThread = std::thread(guarded("control", [&] { control.run(threadsStop); }));
  }
  catch (...) {
    // Same reason as runSimApp: a joinable std::thread destructing calls std::terminate().
    shutdown();
    throw;
  }

  // threadsStop is also how a failing worker asks the others to stop, so watch it here too.
  while (!stop && !framesEnded && !threadsStop) {
    std::this_thread::sleep_for(std::chrono::milliseconds{50});
  }
  shutdown();
  if (failure) {
    throw std::runtime_error(*failure);
  }
  print(framesEnded ? "follow_app: camera stopped delivering frames" : "follow_app: stopped");
}

}  // namespace follow::vision
