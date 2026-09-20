#include <atomic>
#include <csignal>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "MissionProcessor.h"
#include "config/ComponentFactory.h"
#include "config/FileConfigLoader.h"
#include "config/ScenarioLoader.h"

#ifdef FOLLOW_WITH_OPENCV
#include "HwMissionProcessor.h"
#include "interfaces/IFrameSource.h"
#include "providers/PiCameraSource.h"
#endif

namespace {

std::atomic<bool> stopRequested{false};

void onSignal(int)
{
  stopRequested = true;
}

constexpr const char* kUsage =
  "usage: follow_app --sim --scenario FILE [--config FILE] [--link SPEC] [--log FILE]\n"
  "       follow_app --hw [--config FILE] [--link SPEC] [--log FILE]\n"
  "  --sim       simulated camera against ArduPilot SITL or a fake autopilot\n"
  "  --hw        Pi camera + tracker (requires a build with FOLLOW_WITH_OPENCV=ON)\n"
  "  --scenario  scenario JSON (config/scenarios/*.json), --sim only\n"
  "  --config    follow.json (default config/follow.json)\n"
  "  --link      udp:PORT, udp:PORT:HOST:PORT or uart:DEVICE:BAUD (default: udp:14560 for --sim, mavlink.link for --hw)\n"
  "  --log       CSV run log (default follow_run.csv)\n";

}  // namespace

int main(int argc, char** argv)
{
  bool sim = false;
  bool hw = false;
  std::string configPath = "config/follow.json";
  std::string scenarioPath;
  std::optional<std::string> link;
  std::string logPath = "follow_run.csv";
  try {
    for (int i = 1; i < argc; ++i) {
      std::string arg = argv[i];
      auto value = [&]() -> std::string {
        if (i + 1 >= argc) {
          throw std::invalid_argument(arg + " needs a value");
        }
        return argv[++i];
      };
      if (arg == "--sim") {
        sim = true;
      }
      else if (arg == "--hw") {
        hw = true;
      }
      else if (arg == "--scenario") {
        scenarioPath = value();
      }
      else if (arg == "--config") {
        configPath = value();
      }
      else if (arg == "--link") {
        link = value();
      }
      else if (arg == "--log") {
        logPath = value();
      }
      else if (arg == "--help" || arg == "-h") {
        std::cout << kUsage;
        return 0;
      }
      else {
        throw std::invalid_argument("unknown argument " + arg);
      }
    }
    if (sim == hw) {
      throw std::invalid_argument("exactly one of --sim and --hw is required");
    }
    if (sim && scenarioPath.empty()) {
      throw std::invalid_argument("--sim needs --scenario");
    }
#ifndef FOLLOW_WITH_OPENCV
    if (hw) {
      throw std::invalid_argument("--hw needs a build with FOLLOW_WITH_OPENCV=ON");
    }
#endif
  }
  catch (const std::invalid_argument& e) {
    std::cerr << "follow_app: " << e.what() << '\n' << kUsage;
    return 2;
  }

  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);
  // One factory builds every component; nothing below this point names a concrete class.
  follow::config::ComponentFactory factory;
  try {
    if (sim) {
      follow::app::SimAppOptions options;
      options.configPath = configPath;
      options.scenarioPath = scenarioPath;
      options.link = link.value_or(options.link);
      options.logPath = logPath;

      // The scenario comes first: follow.json is parsed with the scenario's config_overrides
      // applied as a merge patch, and the camera model is built from the result.
      std::unique_ptr<follow::config::ScenarioLoader> scenarioLoader = factory.createScenarioLoader(options.scenarioPath);
      scenarioLoader->load();
      std::unique_ptr<follow::interfaces::IConfigLoader> configLoader =
        factory.createConfigLoader(options.configPath, scenarioLoader->getScenario().configOverrides);
      configLoader->load();
      follow::config::AppConfig app = configLoader->getConfig();

      follow::app::MissionProcessor mission(options,
                                            std::move(configLoader),
                                            std::move(scenarioLoader),
                                            factory.createLink(options.link),
                                            factory.createCameraModel(app.camera));
      mission.run(stopRequested, std::cout);
    }
#ifdef FOLLOW_WITH_OPENCV
    else {
      follow::app::HwAppOptions options{.configPath = configPath, .link = link, .logPath = logPath};
      std::unique_ptr<follow::interfaces::IConfigLoader> configLoader = factory.createConfigLoader(options.configPath);
      configLoader->load();
      follow::config::AppConfig app = configLoader->getConfig();

      // Declared before the processor, so cv::VideoCapture::release() runs after run() returns --
      // an end-of-stream release can hang, and HwMissionProcessor's fail-safe must not wait on it.
      std::unique_ptr<follow::interfaces::IFrameSource> frames =
        factory.createFrameSource(follow::providers::PiCameraConfig{.captureWidth = app.vision.captureWidth,
                                                                    .captureHeight = app.vision.captureHeight,
                                                                    .trackWidth = app.vision.trackWidth,
                                                                    .trackHeight = app.vision.trackHeight,
                                                                    .fps = app.vision.fps,
                                                                    .hflip = app.vision.hflip,
                                                                    .vflip = app.vision.vflip});
      follow::app::HwMissionProcessor mission(options,
                                              std::move(configLoader),
                                              factory.createLink(options.link.value_or(app.mavlink.link)),
                                              factory.createCameraModel(app.camera),
                                              factory.createTracker(app.vision.tracker),
                                              *frames);
      mission.run(stopRequested, std::cout);
    }
#endif
  }
  catch (const std::exception& e) {
    std::cerr << "follow_app: " << e.what() << '\n';
    return 1;
  }
  return 0;
}
