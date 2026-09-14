#include "follow/sim/ScenarioRunner.h"

#include <algorithm>
#include <chrono>
#include <cmath>

#include "follow/core/Angles.h"
#include "follow/core/Core.h"

namespace follow::sim {

ScenarioResult runScenario(const core::Config& config,
                           const core::CameraModel& camera,
                           const core::CameraMount& mount,
                           const SimTarget& target,
                           const ScenarioOptions& options)
{
  const core::TimePoint start = core::TimePoint{} + std::chrono::hours{1};
  core::Core followCore(config, camera, mount);
  SyntheticCamera syntheticCamera(camera, mount, options.camera);
  KinematicVehicle vehicle(Pose{.positionNed = {0.0, 0.0, -options.vehicleAltitudeM}}, options.vehicle);
  core::VehicleState vehicleState{};
  std::optional<core::VelocityCmd> setpoint;
  ScenarioResult result;

  auto stepCount = static_cast<int>(std::lround(options.durationS / options.physicsDtS));
  for (int i = 0; i <= stepCount; ++i) {
    double t = i * options.physicsDtS;
    if (i % options.controlEvery == 0) {
      core::TimePoint now = start + std::chrono::duration_cast<core::Clock::duration>(std::chrono::duration<double>(t));
      double targetTime = std::max(0.0, t - options.engageAtS);
      const Pose& pose = vehicle.pose();

      vehicleState.lastHeartbeat = now;
      vehicleState.customMode = t >= options.engageAtS ? core::kModeGuided : core::kModeLoiter;
      vehicleState.attitude.push({.t = now, .roll = pose.roll, .pitch = pose.pitch, .yaw = pose.yaw});

      std::optional<core::TargetObservation> observation = syntheticCamera.step(now, pose, target, targetTime);
      core::Outputs out = followCore.step({.now = now, .vehicle = vehicleState, .target = observation});
      syntheticCamera.handle(out.tracker, now);
      setpoint = out.setpoint;

      core::Vec3 ground = target.positionAt(targetTime);
      core::Vec3 toCenter{ground.x - pose.positionNed.x, ground.y - pose.positionNed.y, -target.height() / 2.0 - pose.positionNed.z};
      StepRecord record{
        .tS = t,
        .state = out.state,
        .setpoint = out.setpoint,
        .targetValid = out.target.valid,
        .trueBearingDeg = core::radToDeg(core::wrapPi(std::atan2(toCenter.y, toCenter.x) - pose.yaw)),
        .trueDistanceM = core::norm(toCenter),
      };
      if (result.states.empty() || result.states.back() != out.state) {
        result.states.push_back(out.state);
      }
      if (out.state == core::State::Following && !result.lockDistanceM) {
        result.lockDistanceM = record.trueDistanceM;
      }
      result.steps.push_back(record);
    }
    vehicle.step(setpoint, options.physicsDtS);
  }
  return result;
}

}  // namespace follow::sim
