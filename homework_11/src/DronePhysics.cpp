#include "DronePhysics.h"
#include "debug.h"
#include "util.h"
#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>

DronePhysics::DronePhysics(const DroneConfig& config)
{
  this->config = config;
  this->speed = 0.f;
  this->timeSecSinceStart = 0.f;
  this->physicsTimeStep = config.physicsTimeStep;
  this->timeScale = config.timeScale;
  this->position = config.startPos;
  this->currentAngle = config.initialDir;
  this->targetAngle = config.initialDir;
  this->attackSpeed = config.attackSpeed;
  this->accel = config.attackSpeed * config.attackSpeed / (2.0f * config.accelPath);
  this->mode = DroneState::STOPPED;
}

DroneTelemetry DronePhysics::getTelemetry() const
{
  std::lock_guard<std::mutex> lock(this->mtx);
  return DroneTelemetry{this->position, this->speed, this->currentAngle, this->accel, this->mode, this->timeSecSinceStart};
}

void DronePhysics::enqueueCommand(const DroneCommand& command)
{
  this->commandQueue.push(command);
}

void DronePhysics::step()
{
  std::lock_guard<std::mutex> lock(this->mtx);

  // (a) Drain the command queue; last command wins.
  DroneCommand cmd{};
  bool found = false;
  while (this->commandQueue.try_pop(cmd)) {
    found = true;
  }
  if (found) {
    this->mode = cmd.mode;
    this->targetAngle = cmd.targetAngle;
  }

  const float dt = this->physicsTimeStep;

  switch (this->mode) {
    case STOPPED:
      // no position change
      break;

    case TURNING:
      util::convergeAngle(this->currentAngle, this->targetAngle, this->config, dt);
      break;

    case ACCELERATING: {
      util::convergeAngle(this->currentAngle, this->targetAngle, this->config, dt);
      float ds = static_cast<float>(this->speed * dt + 0.5f * this->accel * dt * dt);
      Coord dir = {static_cast<float>(cos(this->currentAngle)), static_cast<float>(sin(this->currentAngle))};
      this->speed += this->accel * dt;
      this->position = this->position + dir * ds;
      if (this->speed >= this->attackSpeed) {
        this->speed = this->attackSpeed;
      }
      break;
    }

    case MOVING: {
      util::convergeAngle(this->currentAngle, this->targetAngle, this->config, dt);
      float ds = static_cast<float>(this->speed * dt);
      Coord dir = {static_cast<float>(cos(this->currentAngle)), static_cast<float>(sin(this->currentAngle))};
      this->position = this->position + dir * ds;
      break;
    }

    case DECELERATING: {
      util::convergeAngle(this->currentAngle, this->targetAngle, this->config, dt);
      float ds = static_cast<float>(this->speed * dt - 0.5f * this->accel * dt * dt);
      this->speed -= this->accel * dt;
      if (this->speed <= 0.f) {
        this->speed = 0.f;
      }
      else if (ds > 0.f) {
        Coord dir = {static_cast<float>(cos(this->currentAngle)), static_cast<float>(sin(this->currentAngle))};
        this->position = this->position + dir * ds;
      }
      break;
    }
  }

  // increment the simulation time
  this->timeSecSinceStart += this->physicsTimeStep;
}

bool DronePhysics::isThreadReady()
{
  return this->ready.load();
}

void DronePhysics::run()
{
  LOG("DronePhysics thread up (run() entry)");

  this->ready.store(true);

  while (!this->started.load() && !this->stopFlag.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  // мютекс не потрібен для sleep, бо step() сам блокує доступ до стану дрона
  while (!this->stopFlag.load()) {
    this->step();
    // Спимо без мютекса на час, що відповідає кроку фізики, з урахуванням масштабу часу
    std::this_thread::sleep_for(std::chrono::duration<float>(this->physicsTimeStep / this->timeScale));
  }
}

void DronePhysics::start()
{
  this->started.store(true);
}

void DronePhysics::stop()
{
  this->stopFlag.store(true);
}
