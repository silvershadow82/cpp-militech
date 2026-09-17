#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

#include "FakeAutopilot.h"
#include "follow/config/ConfigJson.h"
#include "follow/core/Types.h"
#include "follow/runtime/HwApp.h"
#include "follow/runtime/RunLog.h"
#include "follow/vision/FrameSource.h"

// Runs the whole follow_app --hw wiring against a fake autopilot with synthetic camera frames for
// about 5 s: the pilot engages after 1 s, the tracker locks on the centered target and follows.
TEST(HwAppTest, EngagesAndFollowsASyntheticTarget)
{
  std::filesystem::path dir = std::filesystem::temp_directory_path() / ("follow_hw_app_test_" + std::to_string(::getpid()));
  std::filesystem::create_directories(dir);
  // No framebuffer in tests: an empty device disables the overlay.
  nlohmann::json configJson = follow::config::readJsonFile(FOLLOW_CONFIG_DIR "/follow.json");
  configJson["vision"]["framebuffer"] = "";
  configJson["camera"] = FOLLOW_CONFIG_DIR "/camera_imx219_160.json";
  std::ofstream(dir / "follow.json") << configJson.dump(2);

  follow::test::FakeAutopilot fc(1.0);
  fc.start();
  follow::vision::SyntheticFrameSource frames(640, 480, follow::core::Clock::now());
  follow::runtime::HwAppOptions options{
    .configPath = dir / "follow.json", .link = "udp:0:127.0.0.1:" + std::to_string(fc.port()), .logPath = dir / "run.csv"};
  std::atomic<bool> stop{false};
  std::thread stopper([&] {
    std::this_thread::sleep_for(std::chrono::seconds{5});
    stop = true;
  });
  std::ostringstream out;

  follow::runtime::runHwApp(options, frames, stop, out);
  stopper.join();
  fc.stop();

  std::ifstream log(options.logPath);
  std::vector<follow::sim::StepRecord> steps = follow::runtime::readRunLog(log);
  auto hasState = [&steps](follow::core::State state) {
    return std::any_of(steps.begin(), steps.end(), [state](const follow::sim::StepRecord& step) { return step.state == state; });
  };
  EXPECT_FALSE(steps.empty()) << out.str();
  EXPECT_TRUE(hasState(follow::core::State::Locking)) << out.str();
  EXPECT_TRUE(hasState(follow::core::State::Following)) << out.str();
  EXPECT_NE(out.str().find("follow_app --hw: tracker kcf"), std::string::npos) << out.str();
  std::filesystem::remove_all(dir);
}
