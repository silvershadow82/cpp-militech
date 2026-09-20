#include "providers/SimVision.h"

#include <algorithm>
#include <chrono>
#include <thread>

#include "sim/ScenarioRunner.h"

namespace follow::providers {

namespace {

constexpr auto kMaxExtrapolation = std::chrono::milliseconds{200};

}  // namespace

SimVision::SimVision(const interfaces::ICameraModel& camera,
                     const models::CameraMount& mount,
                     const sim::SyntheticCameraConfig& cameraConfig,
                     const config::TargetScript& script,
                     double durationS,
                     util::Channels& channels)
  : camera(camera, mount, cameraConfig)
  , script(script)
  , durationS(durationS)
  , channels(channels)
{
}

std::optional<sim::Pose> SimVision::vehiclePose(const control::VehicleState& vehicle, models::TimePoint now)
{
  std::optional<models::AttitudeSample> attitude = vehicle.attitude.latest();
  if (!attitude || !vehicle.position) {
    return std::nullopt;
  }
  const models::LocalPositionNed& local = *vehicle.position;
  auto age = std::clamp(now - local.t, models::Clock::duration::zero(), models::Clock::duration(kMaxExtrapolation));
  double dt = std::chrono::duration<double>(age).count();
  return sim::Pose{.positionNed = {local.position.x + local.velocity.x * dt,
                                   local.position.y + local.velocity.y * dt,
                                   local.position.z + local.velocity.z * dt},
                   .roll = attitude->roll,
                   .pitch = attitude->pitch,
                   .yaw = attitude->yaw};
}

void SimVision::iterate(models::TimePoint now)
{
  std::optional<sim::Pose> pose;
  if (auto vehicle = this->channels.vehicle.read()) {
    pose = vehiclePose(vehicle->value, now);
  }

  for (const control::TrackerRequest& request : this->channels.trackerRequests.drain()) {
    if (request.kind == control::TrackerRequestKind::LockCenter && !this->target) {
      this->lockPending = true;
    }
    this->camera.handle(request, now);
  }
  // Engage on the first iterate where a pose is available, even if that is later than the LockCenter itself.
  if (this->lockPending && !this->target && pose) {
    this->target = this->script.place(*pose);
    this->engagedAt = now;
    this->lockPending = false;
  }
  if (!this->target || !pose) {
    return;
  }

  double targetTimeS = std::chrono::duration<double>(now - this->engagedAt).count();
  if (std::optional<models::TargetObservation> observation = this->camera.step(now, *pose, *this->target, targetTimeS)) {
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
  const auto period = std::chrono::duration_cast<models::Clock::duration>(std::chrono::duration<double>(1.0 / rateHz));
  models::TimePoint next = models::Clock::now();
  while (!stop) {
    this->iterate(models::Clock::now());
    next = std::max(next + period, models::Clock::now());
    std::this_thread::sleep_until(next);
  }
}

}  // namespace follow::providers
