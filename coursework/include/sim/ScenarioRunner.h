#pragma once

#include <optional>
#include <vector>

#include "Types.h"
#include "models/CameraModel.h"
#include "models/Config.h"
#include "models/Frames.h"
#include "sim/KinematicVehicle.h"
#include "sim/SimTarget.h"
#include "sim/SyntheticCamera.h"

namespace follow::sim {

struct ScenarioOptions {
  double durationS{20.0};
  double engageAtS{1.0};  // mode switches LOITER -> GUIDED; the target clock starts here
  double vehicleAltitudeM{2.0};
  double physicsDtS{0.01};
  // Control, attitude and camera run every N physics steps (20 Hz). The synthetic camera is
  // only stepped on these control ticks, so a configured camera fps above the control rate is
  // capped at it, and delivered latency rounds up to the nearest control period (e.g. 80 ms
  // configured -> 100 ms effective at 20 Hz control).
  int controlEvery{5};
  SyntheticCameraConfig camera{};
  KinematicVehicleConfig vehicle{};
};

struct StepRecord {
  double tS{0.0};
  models::State state{models::State::Idle};
  std::optional<models::VelocityCmd> setpoint{};
  bool targetValid{false};
  double trueBearingDeg{0.0};
  double trueDistanceM{0.0};  // camera to target center
};

struct ScenarioResult {
  std::vector<StepRecord> steps;
  std::vector<models::State> states;      // state sequence with consecutive duplicates removed
  std::optional<double> lockDistanceM{};  // true distance when Following was first entered
};

struct GroundTruth {
  double bearingDeg{0.0};  // target center relative to the vehicle heading, positive right
  double distanceM{0.0};   // vehicle to target center
};

GroundTruth groundTruth(const Pose& vehicle, const SimTarget& target, double targetTimeS);

// Runs Core against SyntheticCamera + KinematicVehicle, starting in LOITER with the vehicle
// at the origin facing north.
ScenarioResult runScenario(const models::Config& config,
                           const models::CameraModel& camera,
                           const models::CameraMount& mount,
                           const SimTarget& target,
                           const ScenarioOptions& options);

}  // namespace follow::sim
