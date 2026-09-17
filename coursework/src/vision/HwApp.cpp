#include "follow/runtime/HwApp.h"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <memory>
#include <mutex>
#include <stdexcept>
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

namespace follow::runtime {

void runHwApp(const HwAppOptions& options, vision::IFrameSource& frames, const std::atomic<bool>& stop, std::ostream& out)
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

  std::unique_ptr<vision::FramebufferWriter> framebuffer;
  if (!app.vision.framebuffer.empty()) {
    try {
      framebuffer = std::make_unique<vision::FramebufferWriter>(app.vision.framebuffer);
    }
    catch (const std::runtime_error& e) {
      print(std::string("follow_app: no overlay: ") + e.what());
    }
  }

  Channels channels;
  RunLogWriter log(logFile);
  mavlink::MavlinkIds ids{.sysid = static_cast<uint8_t>(app.mavlink.sysid), .compid = static_cast<uint8_t>(app.mavlink.compid)};
  MavlinkIo io(*link, ids, channels, [&print](const std::string& text) { print("FC: " + text); });
  vision::CameraTrackerSource source(frames,
                                     vision::makeTracker(app.vision.tracker),
                                     vision::CameraTrackerConfig{.reacquirePeriod = std::chrono::milliseconds{app.vision.reacquirePeriodMs},
                                                                 .reacquireExpand = app.vision.reacquireExpand},
                                     channels);
  ControlLoop control(app.core, *camera, app.camera.mount, channels, &log, core::Clock::now());

  print("follow_app --hw: tracker " + app.vision.tracker + ", link " + linkSpec + ", log " + options.logPath.string());
  std::atomic<bool> threadsStop{false};
  std::atomic<bool> framesEnded{false};
  const auto overlayPeriod = std::chrono::duration_cast<core::Clock::duration>(std::chrono::duration<double>(1.0 / app.vision.overlayFps));
  const auto framePeriod = std::chrono::duration_cast<core::Clock::duration>(std::chrono::duration<double>(1.0 / app.vision.fps));

  auto visionLoop = [&] {
    core::TimePoint nextOverlay = core::Clock::now();
    core::TimePoint nextFrame = core::Clock::now();
    while (!threadsStop) {
      if (!source.iterate()) {
        framesEnded = true;
        return;
      }
      core::TimePoint now = core::Clock::now();
      if (framebuffer && now >= nextOverlay && source.lastFrame()) {
        cv::Mat image = source.lastFrame()->image.clone();
        if (auto overlay = channels.overlay.read()) {
          vision::drawOverlay(image, overlay->value);
        }
        framebuffer->write(image);
        nextOverlay = now + overlayPeriod;
      }
      // The Pi camera blocks until the next frame; a file or synthetic source does not, so pace it.
      nextFrame = std::max(nextFrame + framePeriod, now);
      std::this_thread::sleep_until(nextFrame);
    }
  };

  std::thread ioThread;
  std::thread visionThread;
  std::thread controlThread;
  try {
    ioThread = std::thread([&] { io.run(threadsStop); });
    visionThread = std::thread(visionLoop);
    controlThread = std::thread([&] { control.run(threadsStop); });
  }
  catch (...) {
    // Same reason as runSimApp: a joinable std::thread destructing calls std::terminate().
    threadsStop = true;
    if (ioThread.joinable()) {
      ioThread.join();
    }
    if (visionThread.joinable()) {
      visionThread.join();
    }
    if (controlThread.joinable()) {
      controlThread.join();
    }
    throw;
  }

  while (!stop && !framesEnded) {
    std::this_thread::sleep_for(std::chrono::milliseconds{50});
  }
  threadsStop = true;
  controlThread.join();
  visionThread.join();
  ioThread.join();
  print(framesEnded ? "follow_app: camera stopped delivering frames" : "follow_app: stopped");
}

}  // namespace follow::runtime
