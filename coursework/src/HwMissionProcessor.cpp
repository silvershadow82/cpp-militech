#include "HwMissionProcessor.h"
#include "ControlLoop.h"
#include "StatCollector.h"
#include "comms/MavlinkIo.h"
#include "config/FileConfigLoader.h"
#include "providers/CameraTrackerSource.h"
#include "util/Channels.h"
#include "vision/Overlay.h"

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
#include <utility>

namespace follow::app {

namespace detail {

bool shouldDrawOverlay(models::TimePoint now, models::TimePoint &nextOverlay, models::Clock::duration period)
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

HwMissionProcessor::HwMissionProcessor(HwAppOptions options,
                                       std::unique_ptr<interfaces::IConfigLoader> configLoader,
                                       std::unique_ptr<interfaces::IByteLink> link,
                                       std::unique_ptr<interfaces::ICameraModel> camera,
                                       std::unique_ptr<interfaces::ITracker> tracker,
                                       interfaces::IFrameSource &frames)
  : options(std::move(options))
  , configLoader(std::move(configLoader))
  , link(std::move(link))
  , camera(std::move(camera))
  , tracker(std::move(tracker))
  , frames(frames)
{
}

HwMissionProcessor::~HwMissionProcessor() = default;

// Runs once: the tracker is handed to CameraTrackerSource below and is gone afterwards.
void HwMissionProcessor::run(const std::atomic<bool> &stop, std::ostream &out)
{
  config::AppConfig app = this->configLoader->getConfig();
  std::string linkSpec = this->options.link.value_or(app.mavlink.link);
  std::ofstream logFile(this->options.logPath);
  if (!logFile) {
    throw std::runtime_error("cannot write run log " + this->options.logPath.string());
  }

  std::mutex outMutex;
  auto print = [&out, &outMutex](const std::string &line) {
    std::lock_guard<std::mutex> lock(outMutex);
    out << line << std::endl;
  };

  std::unique_ptr<vision::FramebufferWriter> framebuffer;
  if (!app.vision.framebuffer.empty()) {
    try {
      framebuffer = std::make_unique<vision::FramebufferWriter>(app.vision.framebuffer);
    }
    catch (const std::runtime_error &e) {
      print(std::string("follow_app: no overlay: ") + e.what());
    }
  }

  util::Channels channels;
  util::StatCollector log(logFile);
  comms::MavlinkIds ids{.sysid = static_cast<uint8_t>(app.mavlink.sysid), .compid = static_cast<uint8_t>(app.mavlink.compid)};
  comms::MavlinkIo io(*this->link, ids, channels, [&print](const std::string &text) { print("FC: " + text); });
  providers::CameraTrackerSource source(
    this->frames,
    std::move(this->tracker),
    providers::CameraTrackerConfig{.reacquirePeriod = std::chrono::milliseconds{app.vision.reacquirePeriodMs},
                                   .reacquireExpand = app.vision.reacquireExpand},
    channels);
  ControlLoop control(app.core, *this->camera, app.camera.mount, channels, &log, models::Clock::now());

  print("follow_app --hw: tracker " + app.vision.tracker + ", link " + linkSpec + ", log " + this->options.logPath.string());
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

  std::mutex failureMutex;
  std::optional<std::string> failure;
  auto guarded = [&failureMutex, &failure, &threadsStop](const char *name, auto body) {
    return [&failureMutex, &failure, &threadsStop, name, body] {
      try {
        body();
      }
      catch (const std::exception &e) {
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

  auto sendFailsafe = [&] {
    if (!ioThread.joinable()) {
      return;
    }
    if (!ioRunning) {
      print("follow_app: WARNING the zero setpoint was not confirmed sent; the vehicle may hold its last command");
      return;
    }

    channels.setpoint.write(models::VelocityCmd{}, models::Clock::now());
    std::optional<util::Stamped<models::VelocityCmd>> zero = channels.setpoint.read();
    models::TimePoint deadline = models::Clock::now() + kFailsafeSendTimeout;
    while (zero && ioRunning && io.sentSetpointSequence() < zero->sequence && models::Clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds{1});
    }
    if (zero && io.sentSetpointSequence() < zero->sequence) {
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
      struct ClearOnExit {
        std::atomic<bool> &flag;
        ~ClearOnExit() { this->flag = false; }
      } clearOnExit{ioRunning};
      io.run(ioStop);
    }));
    visionThread = std::thread(guarded("vision", visionLoop));
    controlThread = std::thread(guarded("control", [&] { control.run(threadsStop); }));
  }
  catch (...) {
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
  std::string ending = framesEnded ? "follow_app: camera stopped delivering frames" : "follow_app: stopped";
  uint64_t missed = source.missedFrames();
  if (missed > 0) {
    ending += " (" + std::to_string(missed) + (missed == 1 ? " frame" : " frames") + " missed)";
  }
  print(ending);
}

}  // namespace follow::app
