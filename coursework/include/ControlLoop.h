#pragma once

#include <atomic>

#include "StatCollector.h"
#include "control/Core.h"
#include "interfaces/ICameraModel.h"
#include "models/Config.h"
#include "models/Frames.h"
#include "util/Channels.h"

namespace follow::app {

// The 20 Hz control thread: reads the latest vehicle state and observation, steps the core,
// publishes setpoints and tracker requests, and logs the step.
class ControlLoop {
public:
  // `camera` must outlive the loop (Core keeps a reference). `log` may be null.
  ControlLoop(const models::Config& config,
              const interfaces::ICameraModel& camera,
              const models::CameraMount& mount,
              util::Channels& channels,
              util::StatCollector* log,
              models::TimePoint start);

  control::Outputs tick(models::TimePoint now);

  // Ticks at config.rateHz on the steady clock until `stop` is set.
  void run(const std::atomic<bool>& stop);

private:
  models::Config config;
  control::Core core;
  util::Channels& channels;
  util::StatCollector* log;
  models::TimePoint start;
};

}  // namespace follow::app
