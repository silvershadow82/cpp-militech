#include "MissionProcessor.h"
#include "StatCollector.h"
#include "Types.h"
#include "debug.h"
#include "interfaces/IBallisticSolver.h"
#include "interfaces/IConfigLoader.h"
#include "interfaces/ITargetProvider.h"
#include "states/StateStopped.h"
#include "util.h"
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

const int maxApproxSteps{10};
const float maxDiffPrecision{1e-6};

MissionProcessor::MissionProcessor(std::unique_ptr<IBallisticSolver> solver,
                                   std::unique_ptr<ITargetProvider> targetProvider,
                                   std::unique_ptr<IConfigLoader> configLoader,
                                   std::unique_ptr<DronePhysics> dronePhysics)
  : solver(std::move(solver))
  , targetProvider(std::move(targetProvider))
  , configLoader(std::move(configLoader))
  , dronePhysics(std::move(dronePhysics))
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

bool MissionProcessor::computeFireGeometry(const Coord &dronePos,
                                           const Coord &targetPos,
                                           float droneAngle,
                                           const BallisticResult &ballistics)
{
  const float t = ballistics.t;
  const float h = ballistics.h;

  // Drone-to-target distance.
  float distanceToTarget = Coord::distance(dronePos, targetPos);
  if (distanceToTarget <= 0) {
    std::cout << "Invalid D=" << distanceToTarget << std::endl;
    return false;
  }

  // Дивимось, чи потрібна проміжна точка для скидання боєприпасу, щоб дрон встиг розігнатися до необхідної швидкості
  Coord newDronePos = dronePos;
  if (h + this->context.cfg.accelPath > distanceToTarget) {
    if (fabsf(distanceToTarget) < 1e-6) {
      newDronePos.x = targetPos.x - (h + this->context.cfg.accelPath);
      newDronePos.y = targetPos.y;
      distanceToTarget = h + this->context.cfg.accelPath;
    }
    else {
      newDronePos = targetPos - (targetPos - dronePos) * (h + this->context.cfg.accelPath) / distanceToTarget;
      distanceToTarget = Coord::distance(newDronePos, targetPos);
    }
  }

  float ratio = (distanceToTarget - h) / distanceToTarget;
  Coord dir = {static_cast<float>(cos(droneAngle)), static_cast<float>(sin(droneAngle))};

  this->context.dropPoint = newDronePos + (targetPos - newDronePos) * ratio;
  this->context.aimPoint = newDronePos + dir * h;
  this->context.payloadDropTime = t;

  DEBUG("Fire geometry: dropPoint=(" << this->context.dropPoint.x << "," << this->context.dropPoint.y << ")");
  return true;
}

bool MissionProcessor::computeFirePoint(const Target &target)
{
  BallisticResult ballistics = this->solver->solve(this->context.cfg.altitude, this->context.cfg.attackSpeed);

  if (!ballistics.ok) {
    return false;
  }

  Coord targetPos = target.pos;

  float prevDistToFire{INFINITY};
  float prevTimeToFire{INFINITY};
  float distToFire{0.F};
  float timeToFire{0.F};

  int appox{0};

  do {
    // First pass - geometry to the current predicted target position.
    if (!this->computeFireGeometry(this->context.dronePos, targetPos, this->context.droneAngle, ballistics)) {
      return false;
    }
    prevDistToFire = Coord::distance(this->context.dronePos, this->context.dropPoint);
    prevTimeToFire = util::timeToDistance(
      prevDistToFire, this->context.droneSpeed, this->context.cfg.attackSpeed, this->context.droneAccel, this->context.cfg.accelPath);

    // Використовуємо тільки позицію цілі та її швидкість для прогнозування, без врахування майбутньої точки скидання боєприпасу
    targetPos = target.pos + target.velocity * (prevTimeToFire + this->context.payloadDropTime);

    if (!this->computeFireGeometry(this->context.dronePos, targetPos, this->context.droneAngle, ballistics)) {
      return false;
    }
    distToFire = Coord::distance(this->context.dronePos, this->context.dropPoint);
    timeToFire = util::timeToDistance(
      distToFire, this->context.droneSpeed, this->context.cfg.attackSpeed, this->context.droneAccel, this->context.cfg.accelPath);

    // Прохід 2 - прогнозування позиції цілі на основі часу до точки скидання боєприпасу
    targetPos = target.pos + target.velocity * (timeToFire + this->context.payloadDropTime);
    this->context.predictedTargetPos = targetPos;

  } while (appox++ < maxApproxSteps &&
           (fabsf(distToFire - prevDistToFire) > maxDiffPrecision || fabsf(timeToFire - prevTimeToFire) > maxDiffPrecision));

  DEBUG("Converged to firePoint for target=" << target.index << ", maxSteps=" << appox);

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

  if (context.lastTargetIdx != target.index) {
    context.targetAngle = Coord::angle(context.dronePos, context.predictedTargetPos);
    // Якщо дрон змінює ціль, то потрібно врахувати час на розгін/гальмування до нової цілі
    timeToStop = this->context.timeToStop;
  }
  return timeToTarget + timeToStop;
}

void MissionProcessor::updateDroneState()
{
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

  DroneTelemetry t = this->dronePhysics->getTelemetry();

  this->context.dronePos = t.pos;
  this->context.droneSpeed = t.speed;
  this->context.droneAngle = t.angle;

  // Розраховуємо час до всіх цілей, щоб визначити, яка з них буде досягнута першою
  for (int i = 0; i < this->targetProvider->getTargetCount(); i++) {
    auto target = this->targetProvider->getTarget(i);
    auto timeToTarget = this->leadTimeToTarget(target);

    if (timeToTarget < 0) {
      continue;
    }
    timeToTargets[i] = timeToTarget;
  }

  // Оновлюємо індекс останньої цілі, яка буде досягнута першою, та розраховуємо точку скидання боєприпасу
  this->context.lastTargetIdx = util::minValueIdx(timeToTargets);
  auto target = this->targetProvider->getTarget(this->context.lastTargetIdx);

  if (!this->computeFirePoint(target)) {
    return false;
  }

  // Якщо дрон знаходиться на відстані <= 1 м від точки скидання боєприпасу, перевіряємо,
  // чи буде ціль у межах радіусу попадання
  if (Coord::distance(this->context.dronePos, this->context.dropPoint) <= 1.f) {
    auto realTargetPos = target.pos + target.velocity * this->context.payloadDropTime;
    float deviation = Coord::distance(this->context.predictedTargetPos, realTargetPos);
    if (deviation <= this->context.cfg.hitRadius) {
      this->done = true;
    }
  }

  context.targetAngle = Coord::angle(this->context.dronePos, this->context.predictedTargetPos);

  this->updateDroneState();

  // Формуємо команду для фізики дрона на основі поточного стану та бажаного курсу
  this->dronePhysics->enqueueCommand(DroneCommand{this->context.commandMode, this->context.targetAngle});

  // Перечитуємо телеметрію дрона після виконання кроку фізики
  DroneTelemetry t2 = this->dronePhysics->getTelemetry();
  this->context.dronePos = t2.pos;
  this->context.droneSpeed = t2.speed;
  this->context.droneAngle = t2.angle;
  this->context.state = t2.mode;
  this->context.timeSecSinceStart = t2.timeSecSinceStart;

  // Зберемо статистику
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

void MissionProcessor::run()
{
  LOG("MissionProcessor thread up");
  while (!this->stopFlag.load() && this->hasNext()) {
    this->step();
    std::this_thread::sleep_for(std::chrono::duration<double>(this->context.cfg.simTimeStep / this->context.cfg.timeScale));
  }
}

void MissionProcessor::stop()
{
  this->stopFlag.store(true);
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