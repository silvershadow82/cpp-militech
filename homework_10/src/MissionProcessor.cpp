#include "MissionProcessor.h"
#include "StatCollector.h"
#include "Types.h"
#include "debug.h"
#include "interfaces/IBallisticSolver.h"
#include "interfaces/IConfigLoader.h"
#include "interfaces/ITargetProvider.h"
#include "states/StateStopped.h"
#include "util.h"
#include <cmath>
#include <cstring>
#include <iostream>
#include <memory>
#include <vector>

const int maxApproxSteps{10};
const float maxDiffPrecision{1e-6};
// Implementation for the MissionProcessor

MissionProcessor::MissionProcessor(std::unique_ptr<IBallisticSolver> solver,
                                   std::unique_ptr<ITargetProvider> targetProvider,
                                   std::unique_ptr<IConfigLoader> configLoader)
  : solver(std::move(solver))
  , targetProvider(std::move(targetProvider))
  , configLoader(std::move(configLoader))
{
  LOG("Mission processor created successfully.");
}

MissionProcessor::~MissionProcessor() = default;

void MissionProcessor::initContext(const DroneConfig &config, const int targetCount)
{
  this->context = MissionContext{.dronePos = config.startPos,
                                 .cfg = config,
                                 .droneSpeed = 0.f,
                                 .droneAngle = config.initialDir,
                                 .droneAccel = config.attackSpeed * config.attackSpeed / (2 * config.accelPath),
                                 .timeToStop = 0.f,
                                 .step = 0,
                                 .lastTargetIdx = 0,
                                 .t = 0.f};

  this->timeToTargets = std::vector<float>(targetCount);
  this->currentState = std::make_unique<StateStopped>();
  this->done = false;

  LOG("Mission context initialized successfully");
  LOG("droneAccel=" << this->context.droneAccel);
  LOG("dronePos=(" << this->context.dronePos.x << "," << this->context.dronePos.y << ")");
  LOG("droneSpeed=" << this->context.droneSpeed);
}

bool MissionProcessor::computeFirePoint(const Target &target)
{
  auto targetPos = target.pos;

  float prevDistToFire{INFINITY};
  float prevTimeToFire{INFINITY};
  float distToFire{0.F};
  float timeToFire{0.F};

  int appox{0};

  // Converge distance to fire or die trying
  do {
    // First pass: ballistics to current target position
    auto result = this->solver->solve(this->context.dronePos, targetPos, this->context.droneAngle);

    if (!result.ok) {
      return false;
    }
    prevDistToFire = Coord::distance(this->context.dronePos, result.dropPoint);
    prevTimeToFire = util::timeToDistance(
      prevDistToFire, this->context.droneSpeed, this->context.cfg.attackSpeed, this->context.droneAccel, this->context.cfg.accelPath);
    // Reset drone pos and aim at predicted position for second pass
    //   context.dronePos = origDronePos;

    // targetPos = target.at(this->context.t + prevTimeToFire + result.payloadDropTime, this->context.cfg.arrayTimeStep);
    targetPos = target.pos;

    // Second pass: ballistics to predicted position
    result = this->solver->solve(this->context.dronePos, targetPos, this->context.droneAngle);

    if (!result.ok) {
      return false;
    }

    distToFire = Coord::distance(this->context.dronePos, result.dropPoint);
    timeToFire = util::timeToDistance(
      distToFire, this->context.droneSpeed, this->context.cfg.attackSpeed, this->context.droneAccel, this->context.cfg.accelPath);
    // Refine prediction using actual lead time from drone to fireX + payload
    // drop
    // targetPos = target.at(this->context.t + timeToFire + result.payloadDropTime, this->context.cfg.arrayTimeStep);
    targetPos = target.pos;

    this->context.dropPoint = result.dropPoint;
    this->context.payloadDropTime = result.payloadDropTime;
    this->context.aimPoint = result.aimPoint;
    this->context.predictedTargetPos = targetPos;

  } while (appox++ < maxApproxSteps &&
           (fabsf(distToFire - prevDistToFire) > maxDiffPrecision || fabsf(timeToFire - prevTimeToFire) > maxDiffPrecision));

  DEBUG("Converged to firePoint for target=" << target.getIndex() << ", maxSteps=" << appox);

  return true;
}

float MissionProcessor::leadTimeToTarget(const Target &target)
{
  if (!this->computeFirePoint(target)) {
    std::cerr << "Invalid ballistics calculated" << std::endl;
    return -1.F;
  }

  float distanceToFirePoint{Coord::distance(context.dronePos, context.dropPoint)};
  float timeToTarget{util::timeToDistance(
    distanceToFirePoint, this->context.droneSpeed, this->context.cfg.attackSpeed, this->context.droneAccel, this->context.cfg.accelPath)};
  float timeToStop{0.f};

  if (context.lastTargetIdx != target.getIndex()) {
    context.targetAngle = Coord::angle(context.dronePos, context.predictedTargetPos);
    // Switch target and calculate time to stop based on new target angle
    timeToStop = this->context.timeToStop;
  }
  return timeToTarget + timeToStop;
}

void MissionProcessor::updateDroneState()
{
  float ds{0.f};
  Coord dir;

  auto next = this->currentState->execute(this->context);

  if (next) {
    this->currentState = std::move(next);
  }
}

bool MissionProcessor::init(ConfigSource configSource, const std::string &dataFolder)
{
  this->initialized = false;
  this->dataFolder = dataFolder;
  this->statCollector = std::make_unique<StatCollector>(dataFolder + "/simulation.json");

  configLoader->load();
  auto config = configLoader->getConfig();
  auto ammoParams = configLoader->getAmmoParams();

  LOG("read ammoCount=" << configLoader->getAmmoCount());

  if (!ammoParams.contains(config.ammoName)) {
    std::cerr << "Unable to find the selected ammo: " << config.ammoName << '\n';
    return this->initialized;
  }

  auto selectedAmmo = ammoParams.at(config.ammoName);
  auto pp = selectedAmmo.payloadParams();

  this->solver->init(config, pp);
  this->initContext(config, targetProvider->getTargetCount());
  this->initialized = true;

  LOG("DroneConfig loaded attackSpeed=" << this->context.cfg.attackSpeed);

  return this->initialized;
}
bool MissionProcessor::hasNext()
{
  return this->context.step < MAX_STEPS && !this->done;
};
bool MissionProcessor::step()
{
  DEBUG("Simulation at step " << this->context.step);

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
  context.lastTargetIdx = util::minValueIdx(timeToTargets);
  Target target = this->targetProvider->getTarget(context.lastTargetIdx);

  if (!this->computeFirePoint(target)) {
    return false;
  }

  // If drone reached fire point, check payload lands within hitRadius of real
  // target
  if (Coord::distance(this->context.dronePos, this->context.dropPoint) <= 1.f) {
    // Coord realTargetPos = target.at(this->context.t + this->context.payloadDropTime, this->context.cfg.arrayTimeStep);
    // TODO: figure out how to get the real target position at the time of payload drop
    Coord realTargetPos = target.pos;
    float deviation = Coord::distance(this->context.predictedTargetPos, realTargetPos);
    if (deviation <= this->context.cfg.hitRadius) {
      this->done = true;
    }
  }

  context.targetAngle = Coord::angle(this->context.dronePos, this->context.predictedTargetPos);

  // Main state machine
  // Update drone speed, coords and direction
  this->updateDroneState();

  // Collect step stats
  this->statCollector->collectStateStepStats(this->context);

  DEBUG("Step " << this->context.step);
  DEBUG(" |- dronePos=(" << this->context.dronePos.x << "," << this->context.dronePos.y << ")");
  DEBUG(" |- droneSpeed=" << this->context.droneSpeed);
  DEBUG(" |- droneAccel=" << this->context.droneAccel);
  DEBUG(" |- droneAngle=" << this->context.droneAngle);
  DEBUG(" |- dropPoint=(" << this->context.dropPoint.x << "," << this->context.dropPoint.y << ")");
  DEBUG(" |- aimPoint=(" << this->context.aimPoint.x << "," << this->context.aimPoint.y << ")");
  DEBUG(" |- predictedTargetPos=(" << this->context.predictedTargetPos.x << ", " << this->context.predictedTargetPos.y << ")");

  this->context.step++;
  this->context.t += this->context.cfg.simTimeStep;

  return true;
}

void MissionProcessor::reset()
{
  initContext(this->configLoader->getConfig(), this->targetProvider->getTargetCount());
};
void MissionProcessor::changeSolver(std::unique_ptr<IBallisticSolver> solver)
{
  this->solver = std::move(solver);
}
int MissionProcessor::totalSteps()
{
  return this->context.step;
}

void MissionProcessor::printStats()
{
  this->statCollector->printStats();
}