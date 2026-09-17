#pragma once

#include <atomic>

#include "follow/core/CameraModel.h"
#include "follow/core/Config.h"
#include "follow/core/Core.h"
#include "follow/core/Frames.h"
#include "follow/runtime/Channels.h"
#include "follow/runtime/RunLog.h"

namespace follow::runtime {

// The 20 Hz control thread: reads the latest vehicle state and observation, steps the core,
// publishes setpoints and tracker requests, and logs the step.
class ControlLoop {
public:
  // `camera` must outlive the loop (Core keeps a reference). `log` may be null.
  ControlLoop(const core::Config& config,
              const core::CameraModel& camera,
              const core::CameraMount& mount,
              Channels& channels,
              RunLogWriter* log,
              core::TimePoint start);

  core::Outputs tick(core::TimePoint now);

  // Ticks at config.rateHz on the steady clock until `stop` is set.
  void run(const std::atomic<bool>& stop);

private:
  core::Config config;
  core::Core core;
  Channels& channels;
  RunLogWriter* log;
  core::TimePoint start;
};

}  // namespace follow::runtime
