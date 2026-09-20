#include "MissionProcessor.h"

#include <chrono>
#include <cstdint>
#include <fstream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>

#include "StatCollector.h"
#include "comms/LinkSpec.h"
#include "comms/MavlinkIo.h"
#include "config/ConfigJson.h"
#include "config/ScenarioJson.h"
#include "ControlLoop.h"
#include "providers/SimVision.h"
#include "util/Channels.h"

namespace follow::app {

void runSimApp(const SimAppOptions& options, const std::atomic<bool>& stop, std::ostream& out)
{
  config::Scenario scenario = config::loadScenario(options.scenarioPath);
  config::AppConfig app = config::loadAppConfig(options.configPath, scenario.configOverrides);
  // Owned here so it outlives the Core inside ControlLoop, which keeps a reference.
  std::unique_ptr<interfaces::ICameraModel> camera = config::makeCameraModel(app.camera);
  std::unique_ptr<interfaces::IByteLink> link = comms::openLink(comms::parseLinkSpec(options.link));
  std::ofstream logFile(options.logPath);
  if (!logFile) {
    throw std::runtime_error("cannot write run log " + options.logPath.string());
  }

  std::mutex outMutex;
  auto print = [&out, &outMutex](const std::string& line) {
    std::lock_guard<std::mutex> lock(outMutex);
    out << line << std::endl;
  };

  util::Channels channels;
  util::StatCollector log(logFile);
  comms::MavlinkIds ids{.sysid = static_cast<uint8_t>(app.mavlink.sysid), .compid = static_cast<uint8_t>(app.mavlink.compid)};
  comms::MavlinkIo io(*link, ids, channels, [&print](const std::string& text) { print("FC: " + text); });
  providers::SimVision vision(*camera, app.camera.mount, sim::SyntheticCameraConfig{}, scenario.target, scenario.durationS, channels);
  ControlLoop control(app.core, *camera, app.camera.mount, channels, &log, models::Clock::now());

  print("follow_app --sim: scenario " + scenario.name + ", link " + options.link + ", log " + options.logPath.string());
  std::atomic<bool> threadsStop{false};
  std::thread ioThread;
  std::thread visionThread;
  std::thread controlThread;
  try {
    ioThread = std::thread([&] { io.run(threadsStop); });
    visionThread = std::thread([&] { vision.run(threadsStop, options.visionRateHz); });
    controlThread = std::thread([&] { control.run(threadsStop); });
  }
  catch (...) {
    // std::thread's constructor can throw (e.g. resource exhaustion). If it throws after one or two of
    // these have already started, join them here before rethrowing: a std::thread destructs while still
    // joinable calls std::terminate(), which would abort the whole process instead of failing gracefully.
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

  while (!stop && !vision.finished()) {
    std::this_thread::sleep_for(std::chrono::milliseconds{50});
  }
  threadsStop = true;
  controlThread.join();
  visionThread.join();
  ioThread.join();
  print(vision.finished() ? "follow_app: scenario finished" : "follow_app: stopped before the scenario finished");
}

}  // namespace follow::app
