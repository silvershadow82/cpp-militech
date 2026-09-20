#include "control/Supervisor.h"

namespace follow::control {

models::State Supervisor::update(models::TimePoint now,
                                 std::optional<models::TimePoint> lastHeartbeat,
                                 uint32_t customMode,
                                 bool targetValid)
{
  bool fcAlive = lastHeartbeat && now - *lastHeartbeat <= this->config.fcTimeout;
  if (!fcAlive) {
    this->previousMode.reset();
    this->enter(models::State::NoFc, now);
    return this->current;
  }

  bool guided = customMode == models::kModeGuided;
  bool becameGuided = guided && this->previousMode && *this->previousMode != models::kModeGuided;
  this->previousMode = customMode;

  switch (this->current) {
    case models::State::NoFc:
      this->enter(models::State::Idle, now);
      break;
    case models::State::Idle:
      if (becameGuided) {
        this->enter(models::State::Locking, now);
      }
      break;
    case models::State::Locking:
      if (!guided) {
        this->enter(models::State::Idle, now);
      }
      else if (targetValid) {
        this->enter(models::State::Following, now);
      }
      else if (now - this->since >= this->config.lockTimeout) {
        this->enter(models::State::Lost, now);
      }
      break;
    case models::State::Following:
      if (!guided) {
        this->enter(models::State::Idle, now);
      }
      else if (!targetValid) {
        this->enter(models::State::Lost, now);
      }
      break;
    case models::State::Lost:
      if (!guided) {
        this->enter(models::State::Idle, now);
      }
      else if (targetValid) {
        this->enter(models::State::Following, now);
      }
      else if (now - this->since >= this->config.lostTimeout) {
        this->enter(models::State::Hold, now);
      }
      break;
    case models::State::Hold:
      if (!guided) {
        this->enter(models::State::Idle, now);
      }
      break;
  }
  return this->current;
}

void Supervisor::enter(models::State next, models::TimePoint now)
{
  if (next != this->current) {
    this->current = next;
    this->since = now;
  }
}

}  // namespace follow::control
