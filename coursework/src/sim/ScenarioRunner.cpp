#include "sim/ScenarioRunner.h"

#include <algorithm>
#include <chrono>
#include <cmath>

#include "control/Core.h"
#include "models/Angles.h"

namespace follow::sim {

GroundTruth groundTruth(const Pose& vehicle, const SimTarget& target, double targetTimeS)
{
  models::Vec3 ground = target.positionAt(targetTimeS);
  models::Vec3 toCenter{ground.x - vehicle.positionNed.x, ground.y - vehicle.positionNed.y, -target.height() / 2.0 - vehicle.positionNed.z};
  return {.bearingDeg = models::radToDeg(models::wrapPi(std::atan2(toCenter.y, toCenter.x) - vehicle.yaw)),
          .distanceM = models::norm(toCenter)};
}

ScenarioResult runScenario(const models::Config& config,
                           const models::CameraModel& camera,
                           const models::CameraMount& mount,
                           const SimTarget& target,
                           const ScenarioOptions& options)
{
  const models::TimePoint start = models::TimePoint{} + std::chrono::hours{1};
  control::Core followCore(config, camera, mount);
  SyntheticCamera syntheticCamera(camera, mount, options.camera);
  KinematicVehicle vehicle(Pose{.positionNed = {0.0, 0.0, -options.vehicleAltitudeM}}, options.vehicle);
  control::VehicleState vehicleState{};
  std::optional<models::VelocityCmd> setpoint;
  ScenarioResult result;

  auto stepCount = static_cast<int>(std::lround(options.durationS / options.physicsDtS));
  for (int i = 0; i <= stepCount; ++i) {
    double t = i * options.physicsDtS;
    if (i % options.controlEvery == 0) {
      models::TimePoint now = start + std::chrono::duration_cast<models::Clock::duration>(std::chrono::duration<double>(t));
      double targetTime = std::max(0.0, t - options.engageAtS);
      const Pose& pose = vehicle.pose();

      vehicleState.lastHeartbeat = now;
      vehicleState.customMode = t >= options.engageAtS ? models::kModeGuided : models::kModeLoiter;
      vehicleState.attitude.push({.t = now, .roll = pose.roll, .pitch = pose.pitch, .yaw = pose.yaw});

      std::optional<models::TargetObservation> observation = syntheticCamera.step(now, pose, target, targetTime);
      control::Outputs out = followCore.step({.now = now, .vehicle = vehicleState, .target = observation});
      syntheticCamera.handle(out.tracker, now);
      setpoint = out.setpoint;

      GroundTruth truth = groundTruth(pose, target, targetTime);
      StepRecord record{
        .tS = t,
        .state = out.state,
        .setpoint = out.setpoint,
        .targetValid = out.target.valid,
        .trueBearingDeg = truth.bearingDeg,
        .trueDistanceM = truth.distanceM,
      };
      if (result.states.empty() || result.states.back() != out.state) {
        result.states.push_back(out.state);
      }
      if (out.state == models::State::Following && !result.lockDistanceM) {
        result.lockDistanceM = record.trueDistanceM;
      }
      result.steps.push_back(record);
    }
    vehicle.step(setpoint, options.physicsDtS);
  }
  return result;
}

}  // namespace follow::sim
