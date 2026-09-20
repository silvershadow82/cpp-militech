#include <atomic>
#include <csignal>
#include <exception>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>

#include "MissionProcessor.h"

#ifdef FOLLOW_WITH_OPENCV
#include "HwMissionProcessor.h"
#include "config/ConfigJson.h"
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
  try {
    if (sim) {
      follow::app::SimAppOptions options;
      options.configPath = configPath;
      options.scenarioPath = scenarioPath;
      options.link = link.value_or(options.link);
      options.logPath = logPath;
      follow::app::runSimApp(options, stopRequested, std::cout);
    }
#ifdef FOLLOW_WITH_OPENCV
    else {
      follow::config::AppConfig app = follow::config::loadAppConfig(configPath);
      follow::providers::PiCameraSource camera(follow::providers::PiCameraConfig{.captureWidth = app.vision.captureWidth,
                                                                                 .captureHeight = app.vision.captureHeight,
                                                                                 .trackWidth = app.vision.trackWidth,
                                                                                 .trackHeight = app.vision.trackHeight,
                                                                                 .fps = app.vision.fps,
                                                                                 .hflip = app.vision.hflip,
                                                                                 .vflip = app.vision.vflip});
      follow::app::HwAppOptions options{.configPath = configPath, .link = link, .logPath = logPath};
      follow::app::runHwApp(options, camera, stopRequested, std::cout);
    }
#endif
  }
  catch (const std::exception& e) {
    std::cerr << "follow_app: " << e.what() << '\n';
    return 1;
  }
  return 0;
}
