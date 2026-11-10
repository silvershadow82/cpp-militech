#include <exception>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "StatCollector.h"
#include "config/FileConfigLoader.h"
#include "config/ScenarioLoader.h"
#include "sim/ScenarioCheck.h"

namespace {

constexpr const char* kUsage =
  "usage: follow_check_run --log FILE --scenario FILE [--config FILE] [--tolerance-scale X]\n"
  "  Applies the scenario's expectations to a follow_app --sim run log.\n"
  "  --config           follow.json the run used (default config/follow.json)\n"
  "  --tolerance-scale  widens tolerances, bearing and lag limits (default 1.5, for SITL)\n"
  "Exit code: 0 pass, 1 fail, 2 usage or file error.\n";

}  // namespace

int main(int argc, char** argv)
{
  std::string logPath;
  std::string scenarioPath;
  std::string configPath = "config/follow.json";
  double toleranceScale = 1.5;
  try {
    for (int i = 1; i < argc; ++i) {
      std::string arg = argv[i];
      auto value = [&]() -> std::string {
        if (i + 1 >= argc) {
          throw std::invalid_argument(arg + " needs a value");
        }
        return argv[++i];
      };
      if (arg == "--log") {
        logPath = value();
      }
      else if (arg == "--scenario") {
        scenarioPath = value();
      }
      else if (arg == "--config") {
        configPath = value();
      }
      else if (arg == "--tolerance-scale") {
        toleranceScale = std::stod(value());
      }
      else {
        throw std::invalid_argument("unknown argument " + arg);
      }
    }
    if (logPath.empty() || scenarioPath.empty()) {
      throw std::invalid_argument("--log and --scenario are required");
    }

    follow::config::Scenario scenario = follow::config::loadScenario(scenarioPath);
    follow::config::AppConfig config = follow::config::loadAppConfig(configPath, scenario.configOverrides);
    std::ifstream log(logPath);
    if (!log) {
      throw std::runtime_error("cannot open " + logPath);
    }
    std::vector<follow::sim::StepRecord> steps = follow::util::readRunLog(log);
    std::vector<std::string> failures = follow::sim::checkRun(steps, scenario.expect, config.core.control, toleranceScale);

    std::cout << (failures.empty() ? "PASS " : "FAIL ") << scenario.name << " (" << steps.size() << " steps, tolerance x" << toleranceScale
              << ")\n";
    for (const std::string& failure : failures) {
      std::cout << "  " << failure << '\n';
    }
    return failures.empty() ? 0 : 1;
  }
  catch (const std::exception& e) {
    std::cerr << "follow_check_run: " << e.what() << '\n' << kUsage;
    return 2;
  }
}
