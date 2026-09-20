#include "HwMissionProcessor.h"

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

#include "StatCollector.h"
#include "comms/ByteLink.h"
#include "comms/MavlinkIo.h"
#include "config/ConfigJson.h"
#include "ControlLoop.h"
#include "providers/CameraTrackerSource.h"
#include "util/Channels.h"
#include "vision/Overlay.h"
#include "vision/TrackerFactory.h"

namespace follow::app {

namespace detail {

bool shouldDrawOverlay(models::TimePoint now, models::TimePoint& nextOverlay, models::Clock::duration period)
{
  if (now < nextOverlay) {
    return false;
  }
  nextOverlay += period;
  // More than one whole period behind even after advancing once: clamp forward instead of bursting
  // through every slot missed during the stall. Clamping to `now + period`, not `now`, keeps the next
  // draw a full period away -- clamping to `now` alone makes the very next tick (however soon after
  // this one) immediately eligible again, i.e. a back-to-back double draw right after every stall.
  if (nextOverlay < now) {
    nextOverlay = now + period;
  }
  return true;
}

bool shouldReportMissEdge(models::TimePoint now, std::optional<models::TimePoint> lastReport, models::Clock::duration minInterval)
{
  return !lastReport || now - *lastReport >= minInterval;
}

}  // namespace detail

namespace {

// How long shutdown waits for the I/O thread to send the zero setpoint. The thread iterates at
// least every 5 ms, so this is a bound for a link that has already failed, not a normal delay.
constexpr auto kFailsafeSendTimeout = std::chrono::milliseconds{200};

// Minimum gap between two camera-missing/recovered lines. Edge-triggered alone still prints twice
// per drop (missing, then recovered) for every single dropped frame in an alternating drop/good
// pattern -- a marginal link, not just a multi-frame burst -- taking outMutex and flushing at up to
// the frame rate. This bounds total output to at most one line per second regardless of the pattern.
constexpr auto kMissReportInterval = std::chrono::seconds{1};

}  // namespace

void runHwApp(const HwAppOptions& options, interfaces::IFrameSource& frames, const std::atomic<bool>& stop, std::ostream& out)
{
  config::AppConfig app = config::loadAppConfig(options.configPath);
  std::string linkSpec = options.link.value_or(app.mavlink.link);
  // Owned here so it outlives the Core inside ControlLoop, which keeps a reference.
  std::unique_ptr<models::CameraModel> camera = config::makeCameraModel(app.camera);
  std::unique_ptr<comms::ByteLink> link = comms::openLink(comms::parseLinkSpec(linkSpec));
  std::ofstream logFile(options.logPath);
  if (!logFile) {
    throw std::runtime_error("cannot write run log " + options.logPath.string());
  }

  std::mutex outMutex;
  auto print = [&out, &outMutex](const std::string& line) {
    std::lock_guard<std::mutex> lock(outMutex);
    out << line << std::endl;
  };

  std::unique_ptr<vision::FramebufferWriter> framebuffer;
  if (!app.vision.framebuffer.empty()) {
    try {
      framebuffer = std::make_unique<vision::FramebufferWriter>(app.vision.framebuffer);
    }
    catch (const std::runtime_error& e) {
      print(std::string("follow_app: no overlay: ") + e.what());
    }
  }

  util::Channels channels;
  util::RunLogWriter log(logFile);
  comms::MavlinkIds ids{.sysid = static_cast<uint8_t>(app.mavlink.sysid), .compid = static_cast<uint8_t>(app.mavlink.compid)};
  comms::MavlinkIo io(*link, ids, channels, [&print](const std::string& text) { print("FC: " + text); });
  providers::CameraTrackerSource source(
    frames,
    vision::makeTracker(app.vision.tracker),
    providers::CameraTrackerConfig{.reacquirePeriod = std::chrono::milliseconds{app.vision.reacquirePeriodMs},
                                   .reacquireExpand = app.vision.reacquireExpand},
    channels);
  ControlLoop control(app.core, *camera, app.camera.mount, channels, &log, models::Clock::now());

  print("follow_app --hw: tracker " + app.vision.tracker + ", link " + linkSpec + ", log " + options.logPath.string());
  // Two stop flags, not one: the I/O thread outlives the other two so the fail-safe setpoint below
  // can still be put on the wire after the control loop has stopped writing setpoints.
  std::atomic<bool> threadsStop{false};
  std::atomic<bool> ioStop{false};
  std::atomic<bool> framesEnded{false};
  const auto overlayPeriod =
    std::chrono::duration_cast<models::Clock::duration>(std::chrono::duration<double>(1.0 / app.vision.overlayFps));
  const auto framePeriod = std::chrono::duration_cast<models::Clock::duration>(std::chrono::duration<double>(1.0 / app.vision.fps));

  auto visionLoop = [&] {
    models::TimePoint nextOverlay = models::Clock::now();
    models::TimePoint nextFrame = models::Clock::now();
    uint64_t reportedMisses = 0;
    bool missing = false;
    std::optional<models::TimePoint> lastMissReport;  // nullopt: the first edge always reports
    while (!threadsStop) {
      if (!source.iterate()) {
        framesEnded = true;
        return;
      }
      models::TimePoint now = models::Clock::now();
      // One line per dropped frame would take outMutex and flush against the 50 ms frame budget on
      // every miss, and is unbounded over a long flight with intermittent drops. Report the edges,
      // not every frame, the way MavlinkIo::run reports a failing wait -- and rate-limit both edges
      // to at most one line per second (kMissReportInterval, via detail::shouldReportMissEdge), so an
      // alternating drop/good pattern does not still print on every edge forever; edge-triggering
      // alone only bounds output for a multi-frame burst, not a single isolated drop (see
      // ShouldReportMissEdge's tests, which pin exactly this).
      if (source.missedFrames() != reportedMisses) {
        reportedMisses = source.missedFrames();
        if (!missing) {
          missing = true;
          if (detail::shouldReportMissEdge(now, lastMissReport, kMissReportInterval)) {
            lastMissReport = now;
            print("follow_app: camera missing frames");
          }
        }
      }
      else if (missing) {
        missing = false;
        if (detail::shouldReportMissEdge(now, lastMissReport, kMissReportInterval)) {
          lastMissReport = now;
          print("follow_app: camera recovered (" + std::to_string(reportedMisses) + (reportedMisses == 1 ? " frame" : " frames") +
                " missed so far)");
        }
      }
      if (framebuffer && source.lastFrame() && detail::shouldDrawOverlay(now, nextOverlay, overlayPeriod)) {
        cv::Mat image = source.lastFrame()->image.clone();
        if (auto overlay = channels.overlay.read()) {
          vision::drawOverlay(image, overlay->value);
        }
        framebuffer->write(image);
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
      catch (...) {
        // Not everything thrown derives from std::exception, and anything that escapes here reaches
        // std::terminate() -- the one remaining path to a process death with no fail-safe.
        {
          std::lock_guard<std::mutex> lock(failureMutex);
          if (!failure) {
            failure = std::string(name) + " thread failed with an unknown exception";
          }
        }
        threadsStop = true;
      }
    };
  };

  std::thread ioThread;
  std::thread visionThread;
  std::thread controlThread;
  std::atomic<bool> ioRunning{true};  // cleared however the I/O thread leaves, so the fail-safe never waits on a dead one

  // ArduPilot holds the last velocity target until its guided timeout (3 s), so an app that simply
  // stops leaves the vehicle flying at whatever it was last commanded. Command zero and wait for the
  // I/O thread to put it on the wire.
  //
  // This must be able to run while the vision thread is still alive, and must not wait on anything
  // that can block: a stalled libcamerasrc leaves capture.read() parked inside
  // gst_app_sink_pull_sample with no frame, no failure and no end of stream, so the failure budget
  // never counts, ended() never fires and that thread can never be joined. Nothing has touched the
  // capture object here either -- cv::VideoCapture::release() runs in the caller, after runHwApp
  // returns, and an end-of-stream can hang on this build.
  auto sendFailsafe = [&] {
    // Nothing to do if the I/O thread never started at all -- there is no run loop for the zero
    // setpoint to reach either way.
    if (!ioThread.joinable()) {
      return;
    }
    if (!ioRunning) {
      // The thread has already gone (it threw, most likely) before this ran: waiting on it would
      // just burn the whole timeout against a dead thread, and channels.setpoint would be written
      // with nothing left to send it. The zero setpoint is unconditionally unconfirmed in this case
      // -- the same warning the timeout below prints, not silence.
      print("follow_app: WARNING the zero setpoint was not confirmed sent; the vehicle may hold its last command");
      return;
    }
    // The control loop is the only other writer of channels.setpoint (Channels.h), and it has been
    // joined by now, so this is still the last write to the slot.
    channels.setpoint.write(models::VelocityCmd{}, models::Clock::now());
    std::optional<util::Stamped<models::VelocityCmd>> zero = channels.setpoint.read();
    models::TimePoint deadline = models::Clock::now() + kFailsafeSendTimeout;
    while (zero && ioRunning && io.sentSetpointSequence() < zero->sequence && models::Clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds{1});
    }
    if (zero && io.sentSetpointSequence() < zero->sequence) {
      // Silence here would mean the operator and the run log have no record that the vehicle was
      // never told to stop -- the one thing they most need to know after an unexpected exit.
      print("follow_app: WARNING the zero setpoint was not confirmed sent; the vehicle may hold its last command");
    }
  };

  // Every exit path -- a clean stop, a camera that really ended, or a throw -- goes through here.
  auto shutdown = [&] {
    threadsStop = true;
    if (controlThread.joinable()) {
      controlThread.join();
    }
    sendFailsafe();
    if (visionThread.joinable()) {
      visionThread.join();
    }
    if (ioThread.joinable()) {
      ioStop = true;
      ioThread.join();
    }
  };

  try {
    ioThread = std::thread(guarded("mavlink", [&] {
      // Runs on the way out whether run() returns or throws, and before guarded's catch.
      struct ClearOnExit {
        std::atomic<bool>& flag;
        ~ClearOnExit() { this->flag = false; }
      } clearOnExit{ioRunning};
      io.run(ioStop);
    }));
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
  // The miss total goes here because the rate limit can suppress every "recovered" line: a flight of
  // isolated drops each recovering inside kMissReportInterval prints one "missing frames" line and no
  // count at all, and the run log has no miss column either (RunLog.cpp), so this is the only place
  // the operator ever learns how bad the camera was.
  std::string ending = framesEnded ? "follow_app: camera stopped delivering frames" : "follow_app: stopped";
  uint64_t missed = source.missedFrames();
  if (missed > 0) {
    ending += " (" + std::to_string(missed) + (missed == 1 ? " frame" : " frames") + " missed)";
  }
  print(ending);
}

}  // namespace follow::app
