#include "MissionProcessor.h"
#include "StatCollector.h"
#include "Types.h"
#include "debug.h"
#include "interfaces/IBallisticSolver.h"
#include "interfaces/IConfigLoader.h"
#include "interfaces/ITargetProvider.h"
#include "util.h"
#include <cmath>
#include <cstring>
#include <iostream>
#include <vector>

const int maxApproxSteps{10};
const float maxDiffPrecision{1e-6};
// Implementation for the MissionProcessor

MissionProcessor::MissionProcessor(IBallisticSolver *solver,
                                   ITargetProvider *targetProvider,
                                   IConfigLoader *configLoader)
    : solver(solver), targetProvider(targetProvider),
      configLoader(configLoader) {}

void MissionProcessor::initState(const DroneConfig &config,
                                 const int targetCount) {
  this->state = SimState{
      .dronePos = config.startPos,
      .lastTargetIdx = 0,
      .droneSpeed = 0.f,
      .droneAngle = config.initialDir,
      .droneAccel =
          config.attackSpeed * config.attackSpeed / (2 * config.accelPath),
      .step = 0,
      .t = 0.f,
      .droneState = DroneState::STOPPED,
  };

  this->timeToTargets = std::vector<float>(targetCount);
  this->done = false;

  LOG("State initialized.");
  LOG("droneAccel=" << state.droneAccel);
  LOG("dronePos=(" << state.dronePos.x << "," << state.dronePos.y << ")");
  LOG("droneSpeed=" << state.droneSpeed);
}

bool MissionProcessor::computeFirePoint(const Target &target) {
  auto config = this->configLoader->getConfig();
  auto targetPos = target.at(this->state.t, config.arrayTimeStep);

  float prevDistToFire{INFINITY};
  float prevTimeToFire{INFINITY};
  float distToFire{0.F};
  float timeToFire{0.F};

  int appox{0};

  // Converge distance to fire or die trying
  do {
    // First pass: ballistics to current target position
    auto result = this->solver->solve(this->state.dronePos, targetPos,
                                      this->state.droneAngle);

    if (!result.ok) {
      return false;
    }
    prevDistToFire = Coord::distance(this->state.dronePos, result.dropPoint);
    prevTimeToFire = util::timeToDistance(
        prevDistToFire, this->state.droneSpeed, config.attackSpeed,
        this->state.droneAccel, config.accelPath);
    // Reset drone pos and aim at predicted position for second pass
    //   state.dronePos = origDronePos;

    targetPos =
        target.at(this->state.t + prevTimeToFire + result.payloadDropTime,
                  config.arrayTimeStep);

    // Second pass: ballistics to predicted position
    result = this->solver->solve(this->state.dronePos, targetPos,
                                 this->state.droneAngle);

    if (!result.ok) {
      return false;
    }

    distToFire = Coord::distance(this->state.dronePos, result.dropPoint);
    timeToFire = util::timeToDistance(distToFire, this->state.droneSpeed,
                                      config.attackSpeed,
                                      this->state.droneAccel, config.accelPath);
    // Refine prediction using actual lead time from drone to fireX + payload
    // drop
    targetPos = target.at(this->state.t + timeToFire + result.payloadDropTime,
                          config.arrayTimeStep);

    this->state.dropPoint = result.dropPoint;
    this->state.payloadDropTime = result.payloadDropTime;
    this->state.aimPoint = result.aimPoint;
    this->state.predictedTargetPos = targetPos;

  } while (appox++ < maxApproxSteps &&
           (fabsf(distToFire - prevDistToFire) > maxDiffPrecision ||
            fabsf(timeToFire - prevTimeToFire) > maxDiffPrecision));

  DEBUG("Converged to firePoint for target=" << target.getIndex()
                                             << ", maxSteps=" << appox);

  return true;
}

float MissionProcessor::leadTimeToTarget(const Target &target) {
  const DroneConfig config = this->configLoader->getConfig();

  if (!this->computeFirePoint(target)) {
    std::cerr << "Invalid ballistics calculated" << std::endl;
    return -1.F;
  }

  float distanceToFirePoint{Coord::distance(state.dronePos, state.dropPoint)};
  float timeToTarget{util::timeToDistance(distanceToFirePoint, state.droneSpeed,
                                          config.attackSpeed, state.droneAccel,
                                          config.accelPath)};
  float timeToStop{0.f};

  if (state.lastTargetIdx != target.getIndex()) {
    state.targetAngle = Coord::angle(state.dronePos, state.predictedTargetPos);

    switch (state.droneState) {
    case STOPPED:
      timeToStop = 0.f;
      break;
    case ACCELERATING:
    case DECELERATING:
      timeToStop = (config.attackSpeed - state.droneSpeed) / state.droneAccel;
      break;
    case MOVING:
      timeToStop = state.droneSpeed / state.droneAccel;
      break;
    case TURNING: {
      float angleDiff = state.droneAngle - state.targetAngle;
      // normaliza the angle [-PI, PI]
      util::normalizeAngle(angleDiff);
      timeToStop = fabs(angleDiff) / config.angularSpeed;
      break;
    }
    default:
      break;
    }
  }
  return timeToTarget + timeToStop;
}

void MissionProcessor::updateDroneState() {
  float ds{0.f};
  Coord dir;

  const DroneConfig config = this->configLoader->getConfig();

  switch (state.droneState) {
  case STOPPED:
    state.droneState = DroneState::ACCELERATING;
    break;
  case ACCELERATING:
    util::convergeAngle(state.droneAngle, state.targetAngle, config);
    ds = static_cast<float>(state.droneSpeed * config.simTimeStep +
                            0.5f * state.droneAccel * config.simTimeStep *
                                config.simTimeStep);
    state.droneSpeed += state.droneAccel * config.simTimeStep;
    dir = {static_cast<float>(cos(state.droneAngle)),
           static_cast<float>(sin(state.droneAngle))};
    state.dronePos = state.dronePos + dir * ds;

    if (state.droneSpeed >= config.attackSpeed) {
      state.droneSpeed = config.attackSpeed;
      state.droneState = DroneState::MOVING;
    }
    break;
  case DECELERATING:
    util::convergeAngle(state.droneAngle, state.targetAngle, config);
    ds = static_cast<float>(state.droneSpeed * config.simTimeStep -
                            0.5f * state.droneAccel * config.simTimeStep *
                                config.simTimeStep);
    state.droneSpeed -= state.droneAccel * config.simTimeStep;

    if (state.droneSpeed <= 0.f) {
      state.droneSpeed = 0.f;
      state.droneState = DroneState::STOPPED;
    }
    if (ds > 0.f) {
      dir = {static_cast<float>(cos(state.droneAngle)),
             static_cast<float>(sin(state.droneAngle))};
      state.dronePos = state.dronePos + dir * ds;
    }
    break;
  case TURNING: // keep turning until aligned
    if (util::convergeAngle(state.droneAngle, state.targetAngle, config) <
        0.01) {
      state.droneState = DroneState::STOPPED;
    }
    break;
  case MOVING:
    util::convergeAngle(state.droneAngle, state.targetAngle, config);
    ds = static_cast<float>(state.droneSpeed * config.simTimeStep);
    dir = {static_cast<float>(cos(state.droneAngle)),
           static_cast<float>(sin(state.droneAngle))};
    state.dronePos = state.dronePos + dir * ds;
    break;
  default:
    break;
  }
}

bool MissionProcessor::init(ConfigSource configSource,
                            const std::string &dataFolder) {
  this->initialized = false;
  this->dataFolder = dataFolder;
  this->statCollector = new StatCollector(dataFolder + "/simulation.json");

  configLoader->load();
  auto config = configLoader->getConfig();
  auto ammoParams = configLoader->getAmmoParams();

  LOG("read ammoCount=" << configLoader->getAmmoCount());

  if (!ammoParams.contains(config.ammoName)) {
    std::cerr << "Unable to find the selected ammo: " << config.ammoName
              << '\n';
    return this->initialized;
  }

  auto selectedAmmo = ammoParams.at(config.ammoName);
  auto pp = selectedAmmo.payloadParams();

  this->solver->init(config, pp);
  this->initState(config, targetProvider->getTargetCount());
  this->initialized = true;

  LOG("DroneConfig loaded attackSpeed=" << config.attackSpeed);

  return this->initialized;
}
bool MissionProcessor::hasNext() {
  return this->state.step < MAX_STEPS && !this->done;
};
bool MissionProcessor::step() {
  DEBUG("Simulation at step " << state.step);

  const DroneConfig config = this->configLoader->getConfig();

  // Calculate travel time to all the targets
  for (int i = 0; i < this->targetProvider->getTargetCount(); i++) {
    Target target = this->targetProvider->getTarget(i);
    float timeToTarget = this->leadTimeToTarget(target);
    // Skip failed targets
    if (timeToTarget < 0) {
      continue;
    }
    timeToTargets[i] = timeToTarget;
  }

  // Update the target
  state.lastTargetIdx = util::minValueIdx(timeToTargets);
  Target target = this->targetProvider->getTarget(state.lastTargetIdx);

  if (!this->computeFirePoint(target)) {
    return false;
  }

  // If drone reached fire point, check payload lands within hitRadius of real
  // target
  if (Coord::distance(state.dronePos, state.dropPoint) <= 1.f) {
    Coord realTargetPos =
        target.at(state.t + state.payloadDropTime, config.arrayTimeStep);
    float deviation = Coord::distance(state.predictedTargetPos, realTargetPos);
    if (deviation <= config.hitRadius) {
      this->done = true;
    }
  }

  state.targetAngle = Coord::angle(state.dronePos, state.predictedTargetPos);

  // if target changed - update the state to include decelerating/stop
  // if angle between drone and target > turnThreshold

  float angleDiff = state.droneAngle - state.targetAngle;
  // Normalize
  util::normalizeAngle(angleDiff);

  if (fabs(angleDiff) > config.turnThreshold) {
    // Met the threshold - switch to stopping and then turning
    switch (state.droneState) {
    case MOVING:
    case ACCELERATING:
    case DECELERATING:
      state.droneState = DroneState::DECELERATING;
      break;
    case STOPPED:
      state.droneState = DroneState::TURNING;
      break;
    default:
      break;
    }
  }

  // Main state machine
  // Update drone speed, coords and direction
  this->updateDroneState();
  this->statCollector->collectStateStepStats(this->state);

  DEBUG("Step " << state.step);
  DEBUG(" |- dronePos=(" << state.dronePos.x << "," << state.dronePos.y << ")");
  DEBUG(" |- droneSpeed=" << state.droneSpeed);
  DEBUG(" |- droneAccel=" << state.droneAccel);
  DEBUG(" |- droneAngle=" << state.droneAngle);
  DEBUG(" |- dropPoint=(" << state.dropPoint.x << "," << state.dropPoint.y
                          << ")");
  DEBUG(" |- aimPoint=(" << state.aimPoint.x << "," << state.aimPoint.y << ")");
  DEBUG(" |- predictedTargetPos=(" << state.predictedTargetPos.x << ", "
                                   << state.predictedTargetPos.y << ")");

  state.step++;
  state.t += config.simTimeStep;

  return true;
}

void MissionProcessor::reset() {
  initState(this->configLoader->getConfig(),
            this->targetProvider->getTargetCount());
};
void MissionProcessor::changeSolver(IBallisticSolver *solver) {
  this->solver = solver;
}
int MissionProcessor::totalSteps() { return this->state.step; }

void MissionProcessor::printStats() { this->statCollector->printStats(); }

MissionProcessor::~MissionProcessor() {
  delete this->solver;
  delete this->targetProvider;
  delete this->configLoader;
  delete this->statCollector;
};