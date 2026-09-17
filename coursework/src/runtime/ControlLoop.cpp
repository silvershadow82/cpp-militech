#include "follow/runtime/ControlLoop.h"

#include <chrono>
#include <thread>

namespace follow::runtime {

namespace {

// Ground truth older than this is not logged (the simulated camera runs at 20 fps).
constexpr auto kTruthMaxAge = std::chrono::milliseconds{200};

}  // namespace

ControlLoop::ControlLoop(const core::Config& config,
                         const core::CameraModel& camera,
                         const core::CameraMount& mount,
                         Channels& channels,
                         RunLogWriter* log,
                         core::TimePoint start)
  : config(config)
  , core(config, camera, mount)
  , channels(channels)
  , log(log)
  , start(start)
{
}

core::Outputs ControlLoop::tick(core::TimePoint now)
{
  core::Inputs inputs{.now = now};
  if (auto vehicle = this->channels.vehicle.read()) {
    inputs.vehicle = vehicle->value;
  }
  if (auto observation = this->channels.observation.read()) {
    inputs.target = observation->value;
  }

  core::Outputs out = this->core.step(inputs);
  this->channels.overlay.write(out.overlay, now);

  if (out.setpoint) {
    this->channels.setpoint.write(*out.setpoint, now);
  }
  if (out.tracker.kind != core::TrackerRequestKind::None) {
    this->channels.trackerRequests.push(out.tracker);
  }
  if (this->log) {
    LogRow row{.tS = std::chrono::duration<double>(now - this->start).count(),
               .state = out.state,
               .customMode = inputs.vehicle.customMode,
               .target = out.target,
               .setpoint = out.setpoint};
    if (auto truth = this->channels.truth.readFresh(now, kTruthMaxAge)) {
      row.trueBearingDeg = truth->bearingDeg;
      row.trueDistanceM = truth->distanceM;
    }
    this->log->write(row);
  }
  return out;
}

void ControlLoop::run(const std::atomic<bool>& stop)
{
  const auto period = std::chrono::duration_cast<core::Clock::duration>(std::chrono::duration<double>(1.0 / this->config.rateHz));
  core::TimePoint next = core::Clock::now();
  while (!stop) {
    this->tick(core::Clock::now());
    next += period;
    core::TimePoint now = core::Clock::now();
    if (next < now) {
      next = now;  // overran: skip ahead instead of bursting to catch up
    }
    std::this_thread::sleep_until(next);
  }
}

}  // namespace follow::runtime
