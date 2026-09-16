#include "follow/runtime/SimVision.h"

#include <algorithm>
#include <chrono>
#include <thread>

#include "follow/sim/ScenarioRunner.h"

namespace follow::runtime {

namespace {

constexpr auto kMaxExtrapolation = std::chrono::milliseconds{200};

}  // namespace

SimVision::SimVision(const core::CameraModel& camera,
                     const core::CameraMount& mount,
                     const sim::SyntheticCameraConfig& cameraConfig,
                     const config::TargetScript& script,
                     double durationS,
                     Channels& channels)
  : camera(camera, mount, cameraConfig)
  , script(script)
  , durationS(durationS)
  , channels(channels)
{
}

std::optional<sim::Pose> SimVision::vehiclePose(const core::VehicleState& vehicle, core::TimePoint now)
{
  std::optional<core::AttitudeSample> attitude = vehicle.attitude.latest();
  if (!attitude || !vehicle.position) {
    return std::nullopt;
  }
  const core::LocalPositionNed& local = *vehicle.position;
  auto age = std::clamp(now - local.t, core::Clock::duration::zero(), core::Clock::duration(kMaxExtrapolation));
  double dt = std::chrono::duration<double>(age).count();
  return sim::Pose{.positionNed = {local.position.x + local.velocity.x * dt,
                                   local.position.y + local.velocity.y * dt,
                                   local.position.z + local.velocity.z * dt},
                   .roll = attitude->roll,
                   .pitch = attitude->pitch,
                   .yaw = attitude->yaw};
}

void SimVision::iterate(core::TimePoint now)
{
  std::optional<sim::Pose> pose;
  if (auto vehicle = this->channels.vehicle.read()) {
    pose = vehiclePose(vehicle->value, now);
  }

  for (const core::TrackerRequest& request : this->channels.trackerRequests.drain()) {
    if (request.kind == core::TrackerRequestKind::LockCenter && !this->target && pose) {
      this->target = this->script.place(*pose);
      this->engagedAt = now;
    }
    this->camera.handle(request, now);
  }
  if (!this->target || !pose) {
    return;
  }

  double targetTimeS = std::chrono::duration<double>(now - this->engagedAt).count();
  if (std::optional<core::TargetObservation> observation = this->camera.step(now, *pose, *this->target, targetTimeS)) {
    this->channels.observation.write(*observation, now);
  }
  sim::GroundTruth truth = sim::groundTruth(*pose, *this->target, targetTimeS);
  this->channels.truth.write({.bearingDeg = truth.bearingDeg, .distanceM = truth.distanceM}, now);
  if (targetTimeS >= this->durationS) {
    this->done = true;
  }
}

void SimVision::run(const std::atomic<bool>& stop, double rateHz)
{
  const auto period = std::chrono::duration_cast<core::Clock::duration>(std::chrono::duration<double>(1.0 / rateHz));
  core::TimePoint next = core::Clock::now();
  while (!stop) {
    this->iterate(core::Clock::now());
    next = std::max(next + period, core::Clock::now());
    std::this_thread::sleep_until(next);
  }
}

}  // namespace follow::runtime
