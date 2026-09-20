#pragma once

#include <atomic>
#include <filesystem>
#include <memory>
#include <ostream>
#include <string>

#include "interfaces/IByteLink.h"
#include "interfaces/ICameraModel.h"
#include "interfaces/IConfigLoader.h"

namespace follow::config {
class ScenarioLoader;
}

namespace follow::app {

struct SimAppOptions {
  std::filesystem::path configPath{"config/follow.json"};
  std::filesystem::path scenarioPath{};
  std::string link{"udp:14560"};  // ArduPilot SITL connects here with --serialN udpclient:127.0.0.1:14560
  std::filesystem::path logPath{"follow_run.csv"};
  double visionRateHz{100.0};  // SimVision iterations per second; the synthetic camera itself runs at 20 fps
};

// follow_app --sim: runs the MAVLink I/O, simulated vision and control threads until the scenario
// duration has passed after engage or `stop` is set. Progress lines and FC STATUSTEXT go to `out`.
//
// The dependencies are built by config::ComponentFactory and injected here; the loaders must
// already have been load()ed, because the camera model and the link are derived from what they
// read. run() throws std::runtime_error for an unwritable log path.
class MissionProcessor {
public:
  MissionProcessor(SimAppOptions options,
                   std::unique_ptr<interfaces::IConfigLoader> configLoader,
                   std::unique_ptr<config::ScenarioLoader> scenarioLoader,
                   std::unique_ptr<interfaces::IByteLink> link,
                   std::unique_ptr<interfaces::ICameraModel> camera);
  ~MissionProcessor();

  void run(const std::atomic<bool>& stop, std::ostream& out);

private:
  SimAppOptions options;
  std::unique_ptr<interfaces::IConfigLoader> configLoader;
  std::unique_ptr<config::ScenarioLoader> scenarioLoader;
  std::unique_ptr<interfaces::IByteLink> link;
  // Outlives the Core inside ControlLoop, which keeps a reference.
  std::unique_ptr<interfaces::ICameraModel> camera;
};

}  // namespace follow::app
