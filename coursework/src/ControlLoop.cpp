#include "ControlLoop.h"

#include <chrono>
#include <thread>

namespace follow::app {

namespace {

// Ground truth older than this is not logged (the simulated camera runs at 20 fps).
constexpr auto kTruthMaxAge = std::chrono::milliseconds{200};

}  // namespace

ControlLoop::ControlLoop(const models::Config& config,
                         const interfaces::ICameraModel& camera,
                         const models::CameraMount& mount,
                         util::Channels& channels,
                         util::RunLogWriter* log,
                         models::TimePoint start)
  : config(config)
  , core(config, camera, mount)
  , channels(channels)
  , log(log)
  , start(start)
{
}

control::Outputs ControlLoop::tick(models::TimePoint now)
{
  control::Inputs inputs{.now = now};
  if (auto vehicle = this->channels.vehicle.read()) {
    inputs.vehicle = vehicle->value;
  }
  if (auto observation = this->channels.observation.read()) {
    inputs.target = observation->value;
  }

  control::Outputs out = this->core.step(inputs);
  this->channels.overlay.write(out.overlay, now);

  if (out.setpoint) {
    this->channels.setpoint.write(*out.setpoint, now);
  }
  if (out.tracker.kind != control::TrackerRequestKind::None) {
    this->channels.trackerRequests.push(out.tracker);
  }
  if (this->log) {
    util::LogRow row{.tS = std::chrono::duration<double>(now - this->start).count(),
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
  const auto period = std::chrono::duration_cast<models::Clock::duration>(std::chrono::duration<double>(1.0 / this->config.rateHz));
  models::TimePoint next = models::Clock::now();
  while (!stop) {
    this->tick(models::Clock::now());
    next += period;
    models::TimePoint now = models::Clock::now();
    if (next < now) {
      next = now;  // overran: skip ahead instead of bursting to catch up
    }
    std::this_thread::sleep_until(next);
  }
}

}  // namespace follow::app
