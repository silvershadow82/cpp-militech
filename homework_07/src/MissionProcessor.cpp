#include "debug.hpp"
#include "common.hpp"
#include "ComponentFactory.hpp"
#include "MissionProcessor.hpp"
#include "util.hpp"
#include "nlohmann/json.hpp"
#include "nlohmann/json_fwd.hpp"
#include <cmath>
#include <cstring>
#include <iostream>
#include <fstream>

using json = nlohmann::json;

const int maxApproxSteps{10};
const float maxDiffPrecision{1e-6};
// Implementation for the MissionProcessor

void MissionProcessor::initState(const DroneConfig& config, const int targetCount)
{
  state = SimState{};
  state.dronePos = config.startPos;
  state.droneSpeed = 0.f;
  state.droneAngle = config.initialDir;
  state.droneAccel = config.attackSpeed * config.attackSpeed / (2 * config.accelPath);
  state.droneState = DroneState::STOPPED;
  state.lastTargetIdx = 0;
  state.step = 0;
  state.t = 0.f;

  statSteps = new SimStep[MAX_STEPS];
  timeToTargets = new float[targetCount]{};
  this->done = false;

  LOG("State initialized.");
  LOG("droneAccel=" << state.droneAccel);
  LOG("dronePos=(" << state.dronePos.x << "," << state.dronePos.y << ")");
  LOG("droneSpeed=" << state.droneSpeed);
}

bool MissionProcessor::computeFirePoint(const Target& target)
{
  DroneConfig config = this->configLoader->getConfig();
  Coord targetPos = target.at(this->state.t, config.arrayTimeStep);

  float prevDistToFire{INFINITY};
  float prevTimeToFire{INFINITY};
  float distToFire{0.F};
  float timeToFire{0.F};

  int appox{0};

  // Converge distance to fire or die trying
  do {
    // First pass: ballistics to current target position
    BallisticResult result = this->solver->solve(this->state.dronePos, targetPos, this->state.droneAngle);

    if (!result.ok) {
      return false;
    }
    prevDistToFire = Coord::distance(this->state.dronePos, result.dropPoint);
    prevTimeToFire = timeToDistance(prevDistToFire, this->state.droneSpeed, config.attackSpeed, this->state.droneAccel, config.accelPath);
    // Reset drone pos and aim at predicted position for second pass
    //   state.dronePos = origDronePos;

    targetPos = target.at(this->state.t + prevTimeToFire + result.payloadDropTime, config.arrayTimeStep);

    // Second pass: ballistics to predicted position
    result = this->solver->solve(this->state.dronePos, targetPos, this->state.droneAngle);

    if (!result.ok) {
      return false;
    }

    distToFire = Coord::distance(this->state.dronePos, result.dropPoint);
    timeToFire = timeToDistance(distToFire, this->state.droneSpeed, config.attackSpeed, this->state.droneAccel, config.accelPath);
    // Refine prediction using actual lead time from drone to fireX + payload drop
    targetPos = target.at(this->state.t + timeToFire + result.payloadDropTime, config.arrayTimeStep);

    this->state.dropPoint = result.dropPoint;
    this->state.payloadDropTime = result.payloadDropTime;
    this->state.aimPoint = result.aimPoint;
    this->state.predictedTargetPos = targetPos;

  } while (appox++ < maxApproxSteps &&
           (fabsf(distToFire - prevDistToFire) > maxDiffPrecision || fabsf(timeToFire - prevTimeToFire) > maxDiffPrecision));

  DEBUG("Converged to firePoint for target=" << target.getIndex() << ", maxSteps=" << appox);

  return true;
}

float MissionProcessor::leadTimeToTarget(const Target& target)
{
  const DroneConfig config = this->configLoader->getConfig();

  if (!computeFirePoint(target)) {
    std::cerr << "Invalid ballistics calculated" << std::endl;
    return -1.F;
  }

  float distanceToFirePoint{Coord::distance(state.dronePos, state.dropPoint)};
  float timeToTarget{timeToDistance(distanceToFirePoint, state.droneSpeed, config.attackSpeed, state.droneAccel, config.accelPath)};
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
        normalizeAngle(angleDiff);
        timeToStop = fabs(angleDiff) / config.angularSpeed;
        break;
      }
      default:
        break;
    }
  }
  return timeToTarget + timeToStop;
}

void MissionProcessor::updateDroneState()
{
  float ds{0.f};
  Coord dir;

  const DroneConfig config = this->configLoader->getConfig();

  switch (state.droneState) {
    case STOPPED:
      state.droneState = DroneState::ACCELERATING;
      break;
    case ACCELERATING:
      convergeAngle(state.droneAngle, state.targetAngle, config);
      ds = static_cast<float>(state.droneSpeed * config.simTimeStep + 0.5f * state.droneAccel * config.simTimeStep * config.simTimeStep);
      state.droneSpeed += state.droneAccel * config.simTimeStep;
      dir = {static_cast<float>(cos(state.droneAngle)), static_cast<float>(sin(state.droneAngle))};
      state.dronePos = state.dronePos + dir * ds;

      if (state.droneSpeed >= config.attackSpeed) {
        state.droneSpeed = config.attackSpeed;
        state.droneState = DroneState::MOVING;
      }
      break;
    case DECELERATING:
      convergeAngle(state.droneAngle, state.targetAngle, config);
      ds = static_cast<float>(state.droneSpeed * config.simTimeStep - 0.5f * state.droneAccel * config.simTimeStep * config.simTimeStep);
      state.droneSpeed -= state.droneAccel * config.simTimeStep;

      if (state.droneSpeed <= 0.f) {
        state.droneSpeed = 0.f;
        state.droneState = DroneState::STOPPED;
      }
      if (ds > 0.f) {
        dir = {static_cast<float>(cos(state.droneAngle)), static_cast<float>(sin(state.droneAngle))};
        state.dronePos = state.dronePos + dir * ds;
      }
      break;
    case TURNING:  // keep turning until aligned
      if (convergeAngle(state.droneAngle, state.targetAngle, config) < 0.01) {
        state.droneState = DroneState::STOPPED;
      }
      break;
    case MOVING:
      convergeAngle(state.droneAngle, state.targetAngle, config);
      ds = static_cast<float>(state.droneSpeed * config.simTimeStep);
      dir = {static_cast<float>(cos(state.droneAngle)), static_cast<float>(sin(state.droneAngle))};
      state.dronePos = state.dronePos + dir * ds;
      break;
    default:
      break;
  }
}

void MissionProcessor::collectCurrentStepStats()
{
  statSteps[state.step].pos = state.dronePos;
  statSteps[state.step].direction = state.droneAngle;
  statSteps[state.step].targetIdx = state.lastTargetIdx;
  statSteps[state.step].dropPoint = state.dropPoint;
  statSteps[state.step].state = state.droneState;
  statSteps[state.step].predictedTarget = state.predictedTargetPos;
  statSteps[state.step].aimPoint = state.aimPoint;
}

bool MissionProcessor::init(ConfigSource configSource, const std::string& dataFolder)
{
  this->initialized = false;
  this->dataFolder = dataFolder;
  ComponentFactory componentFactory = ComponentFactory();

  switch (configSource) {
    case ConfigSource::FILE:
      configLoader = componentFactory.createLoader(LoaderType::FILE, dataFolder);
      solver = componentFactory.createSolver(SolverType::ANALYTICAL);
      targetProvider = componentFactory.createProvider(ProviderType::JSON, dataFolder + "/targets.json");
      break;
    default:
      break;
  }

  if (configLoader) {
    configLoader->load();
    DroneConfig config = configLoader->getConfig();
    AmmoParams* ammoParams = configLoader->getAmmoParams();

    LOG("read ammoCount=" << configLoader->getAmmoCount());

    std::optional<AmmoParams> selectedAmmo = AmmoParams::ammoByName(config.ammoName, ammoParams, configLoader->getAmmoCount());
    if (!selectedAmmo.has_value()) {
      std::cerr << "Unable to find the selected ammo: " << config.ammoName << '\n';
      return this->initialized;
    }
    PayloadParams pp = selectedAmmo->payloadParams();
    solver->init(config, pp);
    this->initState(config, targetProvider->getTargetCount());
    this->initialized = true;

    LOG("DroneConfig loaded attackSpeed=" << config.attackSpeed);
  }
  return this->initialized;
}
bool MissionProcessor::hasNext()
{
  return this->state.step < MAX_STEPS && !this->done;
};
bool MissionProcessor::step()
{
  DEBUG("Simulation at step " << state.step);

  const DroneConfig config = this->configLoader->getConfig();

  // Calculate travel time to all the targets
  for (int i = 0; i < targetProvider->getTargetCount(); i++) {
    Target target = targetProvider->getTarget(i);
    float timeToTarget = leadTimeToTarget(target);
    // Skip failed targets
    if (timeToTarget < 0) {
      continue;
    }
    timeToTargets[i] = timeToTarget;
  }

  // Update the target
  state.lastTargetIdx = minValueIdx(timeToTargets, targetProvider->getTargetCount());
  Target target = targetProvider->getTarget(state.lastTargetIdx);

  if (!computeFirePoint(target)) {
    return false;
  }

  // If drone reached fire point, check payload lands within hitRadius of real target
  if (Coord::distance(state.dronePos, state.dropPoint) <= 1.f) {
    Coord realTargetPos = target.at(state.t + state.payloadDropTime, config.arrayTimeStep);
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
  normalizeAngle(angleDiff);

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

  this->collectCurrentStepStats();

  DEBUG("Step " << state.step);
  DEBUG(" |- dronePos=(" << state.dronePos.x << "," << state.dronePos.y << ")");
  DEBUG(" |- droneSpeed=" << state.droneSpeed);
  DEBUG(" |- droneAccel=" << state.droneAccel);
  DEBUG(" |- droneAngle=" << state.droneAngle);
  DEBUG(" |- dropPoint=(" << state.dropPoint.x << "," << state.dropPoint.y << ")");
  DEBUG(" |- aimPoint=(" << state.aimPoint.x << "," << state.aimPoint.y << ")");
  DEBUG(" |- predictedTargetPos=(" << state.predictedTargetPos.x << ", " << state.predictedTargetPos.y << ")");

  state.step++;
  state.t += config.simTimeStep;

  return true;
}

void MissionProcessor::reset()
{
  initState(this->configLoader->getConfig(), this->targetProvider->getTargetCount());
};
void MissionProcessor::changeSolver(IBallisticSolver* solver)
{
  this->solver = solver;
}
int MissionProcessor::totalSteps()
{
  return this->state.step;
}
void MissionProcessor::printStats()
{
  std::ofstream output(dataFolder + "/simulation.json");

  if (!output.is_open()) {
    std::cerr << "Unable to open simulation.json" << '\n';
    return;
  }

  json out{};
  out["totalSteps"] = this->state.step;
  out["steps"] = json::array();

  for (int i = 0; i < this->state.step; i++) {
    json step{};
    step["position"] = {{"x", this->statSteps[i].pos.x}, {"y", this->statSteps[i].pos.y}};
    step["direction"] = this->statSteps[i].direction;
    step["state"] = this->statSteps[i].state;
    step["targetIndex"] = this->statSteps[i].targetIdx;
    step["dropPoint"] = {{"x", this->statSteps[i].dropPoint.x}, {"y", this->statSteps[i].dropPoint.y}};
    step["aimPoint"] = {{"x", this->statSteps[i].aimPoint.x}, {"y", this->statSteps[i].aimPoint.y}};
    step["predictedTarget"] = {{"x", this->statSteps[i].predictedTarget.x}, {"y", this->statSteps[i].predictedTarget.y}};
    out["steps"].push_back(step);
  }

  output << out.dump(2);

  output.close();
}

MissionProcessor::~MissionProcessor()
{
  delete this->solver;
  delete this->targetProvider;
  delete this->configLoader;
  delete[] this->statSteps;
  delete[] this->timeToTargets;
};