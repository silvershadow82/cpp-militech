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
#include "follow/runtime/HwApp.h"
#include "follow/vision/FrameSource.h"

namespace {

// The state column (second field) of every data row of a run log.
std::vector<std::string> loggedStates(const std::filesystem::path& path)
{
  std::ifstream in(path);
  std::string line;
  std::getline(in, line);  // header
  std::vector<std::string> states;
  while (std::getline(in, line)) {
    std::size_t first = line.find(',');
    std::size_t second = line.find(',', first + 1);
    states.push_back(line.substr(first + 1, second - first - 1));
  }
  return states;
}

}  // namespace

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

  std::vector<std::string> states = loggedStates(options.logPath);
  ASSERT_FALSE(states.empty()) << out.str();
  EXPECT_NE(std::find(states.begin(), states.end(), "Locking"), states.end()) << out.str();
  EXPECT_NE(std::find(states.begin(), states.end(), "Following"), states.end()) << out.str();
  EXPECT_NE(out.str().find("follow_app --hw: tracker kcf"), std::string::npos) << out.str();
  std::filesystem::remove_all(dir);
}
