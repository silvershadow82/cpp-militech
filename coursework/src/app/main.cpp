#include <atomic>
#include <csignal>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>

#include "follow/runtime/SimApp.h"

namespace {

std::atomic<bool> stopRequested{false};

void onSignal(int)
{
  stopRequested = true;
}

constexpr const char* kUsage =
  "usage: follow_app --sim --scenario FILE [--config FILE] [--link SPEC] [--log FILE]\n"
  "  --scenario  scenario JSON (config/scenarios/*.json)\n"
  "  --config    follow.json (default config/follow.json)\n"
  "  --link      udp:PORT, udp:PORT:HOST:PORT or uart:DEVICE:BAUD (default udp:14560)\n"
  "  --log       CSV run log (default follow_run.csv)\n"
  "Hardware mode (--hw) arrives with the camera and tracker adapter.\n";

}  // namespace

int main(int argc, char** argv)
{
  follow::runtime::SimAppOptions options;
  bool sim = false;
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
      else if (arg == "--scenario") {
        options.scenarioPath = value();
      }
      else if (arg == "--config") {
        options.configPath = value();
      }
      else if (arg == "--link") {
        options.link = value();
      }
      else if (arg == "--log") {
        options.logPath = value();
      }
      else if (arg == "--help" || arg == "-h") {
        std::cout << kUsage;
        return 0;
      }
      else {
        throw std::invalid_argument("unknown argument " + arg);
      }
    }
    if (!sim || options.scenarioPath.empty()) {
      throw std::invalid_argument("--sim and --scenario are required");
    }
  }
  catch (const std::invalid_argument& e) {
    std::cerr << "follow_app: " << e.what() << '\n' << kUsage;
    return 2;
  }

  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);
  try {
    follow::runtime::runSimApp(options, stopRequested, std::cout);
  }
  catch (const std::exception& e) {
    std::cerr << "follow_app: " << e.what() << '\n';
    return 1;
  }
  return 0;
}
