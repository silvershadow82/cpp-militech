#include "MissionProcessor.h"

#include <chrono>
#include <cstdint>
#include <fstream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

#include "ControlLoop.h"
#include "StatCollector.h"
#include "comms/MavlinkIo.h"
#include "config/FileConfigLoader.h"
#include "config/ScenarioLoader.h"
#include "providers/SimVision.h"
#include "util/Channels.h"

namespace follow::app {

MissionProcessor::MissionProcessor(SimAppOptions options,
                                   std::unique_ptr<interfaces::IConfigLoader> configLoader,
                                   std::unique_ptr<config::ScenarioLoader> scenarioLoader,
                                   std::unique_ptr<interfaces::IByteLink> link,
                                   std::unique_ptr<interfaces::ICameraModel> camera)
  : options(std::move(options))
  , configLoader(std::move(configLoader))
  , scenarioLoader(std::move(scenarioLoader))
  , link(std::move(link))
  , camera(std::move(camera))
{
}

MissionProcessor::~MissionProcessor() = default;

void MissionProcessor::run(const std::atomic<bool>& stop, std::ostream& out)
{
  config::Scenario scenario = this->scenarioLoader->getScenario();
  config::AppConfig app = this->configLoader->getConfig();
  std::ofstream logFile(this->options.logPath);
  if (!logFile) {
    throw std::runtime_error("cannot write run log " + this->options.logPath.string());
  }

  std::mutex outMutex;
  auto print = [&out, &outMutex](const std::string& line) {
    std::lock_guard<std::mutex> lock(outMutex);
    out << line << std::endl;
  };

  util::Channels channels;
  util::StatCollector log(logFile);
  comms::MavlinkIds ids{.sysid = static_cast<uint8_t>(app.mavlink.sysid), .compid = static_cast<uint8_t>(app.mavlink.compid)};
  comms::MavlinkIo io(*this->link, ids, channels, [&print](const std::string& text) { print("FC: " + text); });
  providers::SimVision vision(*this->camera, app.camera.mount, sim::SyntheticCameraConfig{}, scenario.target, scenario.durationS, channels);
  ControlLoop control(app.core, *this->camera, app.camera.mount, channels, &log, models::Clock::now());

  print("follow_app --sim: scenario " + scenario.name + ", link " + this->options.link + ", log " + this->options.logPath.string());
  std::atomic<bool> threadsStop{false};
  std::thread ioThread;
  std::thread visionThread;
  std::thread controlThread;
  try {
    ioThread = std::thread([&] { io.run(threadsStop); });
    visionThread = std::thread([&] { vision.run(threadsStop, this->options.visionRateHz); });
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
