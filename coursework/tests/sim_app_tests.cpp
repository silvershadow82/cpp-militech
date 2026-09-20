#include <gtest/gtest.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "FakeAutopilot.h"
#include "MissionProcessor.h"
#include "StatCollector.h"
#include "config/ConfigJson.h"
#include "config/ScenarioJson.h"
#include "sim/ScenarioCheck.h"

using namespace follow;

namespace {

std::string joined(const std::vector<std::string>& lines)
{
  std::string text;
  for (const std::string& line : lines) {
    text += "\n  " + line;
  }
  return text;
}

}  // namespace

// Runs the whole follow_app --sim wiring (threads, UDP, MAVLink, config, run log) against a fake
// autopilot for about 8 s of wall-clock time, then checks the log like follow_check_run does.
TEST(SimAppTest, StationaryScenarioPassesAgainstFakeAutopilot)
{
  // Setup: the committed stationary scenario shortened to 5 s after engage
  std::filesystem::path dir = std::filesystem::temp_directory_path() / ("follow_sim_app_test_" + std::to_string(::getpid()));
  std::filesystem::create_directories(dir);
  nlohmann::json scenarioJson = config::readJsonFile(FOLLOW_CONFIG_DIR "/scenarios/stationary.json");
  scenarioJson["duration_s"] = 5.0;
  std::ofstream(dir / "stationary.json") << scenarioJson.dump(2);

  test::FakeAutopilot fc(1.0);
  fc.start();
  app::SimAppOptions options{.configPath = FOLLOW_CONFIG_DIR "/follow.json",
                             .scenarioPath = dir / "stationary.json",
                             .link = "udp:0:127.0.0.1:" + std::to_string(fc.port()),
                             .logPath = dir / "run.csv"};
  std::atomic<bool> stop{false};
  std::atomic<bool> finished{false};
  std::thread watchdog([&] {
    for (int i = 0; i < 300 && !finished; ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds{100});
    }
    stop = true;  // 30 s without finishing: stop instead of hanging the test run
  });
  std::ostringstream out;

  // Run
  app::runSimApp(options, stop, out);
  finished = true;
  watchdog.join();
  fc.stop();

  // Assert
  EXPECT_NE(out.str().find("follow_app: scenario finished"), std::string::npos) << out.str();
  std::ifstream log(options.logPath);
  std::vector<sim::StepRecord> steps = util::readRunLog(log);
  config::Scenario scenario = config::loadScenario(options.scenarioPath);
  config::AppConfig app = config::loadAppConfig(options.configPath, scenario.configOverrides);
  std::vector<std::string> failures = sim::checkRun(steps, scenario.expect, app.core.control, 1.5);
  EXPECT_TRUE(failures.empty()) << joined(failures);
  std::filesystem::remove_all(dir);
}
