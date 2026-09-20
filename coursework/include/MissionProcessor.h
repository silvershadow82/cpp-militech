#pragma once

#include <atomic>
#include <filesystem>
#include <ostream>
#include <string>

namespace follow::runtime {

struct SimAppOptions {
  std::filesystem::path configPath{"config/follow.json"};
  std::filesystem::path scenarioPath{};
  std::string link{"udp:14560"};  // ArduPilot SITL connects here with --serialN udpclient:127.0.0.1:14560
  std::filesystem::path logPath{"follow_run.csv"};
  double visionRateHz{100.0};  // SimVision iterations per second; the synthetic camera itself runs at 20 fps
};

// follow_app --sim: loads follow.json with the scenario's overrides, then runs the MAVLink I/O,
// simulated vision and control threads until the scenario duration has passed after engage or
// `stop` is set. Progress lines and FC STATUSTEXT go to `out`.
// Throws config::ConfigError or std::runtime_error for bad files, links or log paths.
void runSimApp(const SimAppOptions& options, const std::atomic<bool>& stop, std::ostream& out);

}  // namespace follow::runtime
