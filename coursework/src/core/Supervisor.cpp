#include "follow/core/Supervisor.h"

namespace follow::core {

State Supervisor::update(TimePoint now, std::optional<TimePoint> lastHeartbeat, uint32_t customMode, bool targetValid)
{
  bool fcAlive = lastHeartbeat && now - *lastHeartbeat <= this->config.fcTimeout;
  if (!fcAlive) {
    this->previousMode.reset();
    this->enter(State::NoFc, now);
    return this->current;
  }

  bool guided = customMode == kModeGuided;
  bool becameGuided = guided && this->previousMode && *this->previousMode != kModeGuided;
  this->previousMode = customMode;

  switch (this->current) {
    case State::NoFc:
      this->enter(State::Idle, now);
      break;
    case State::Idle:
      if (becameGuided) {
        this->enter(State::Locking, now);
      }
      break;
    case State::Locking:
      if (!guided) {
        this->enter(State::Idle, now);
      }
      else if (targetValid) {
        this->enter(State::Following, now);
      }
      else if (now - this->since >= this->config.lockTimeout) {
        this->enter(State::Lost, now);
      }
      break;
    case State::Following:
      if (!guided) {
        this->enter(State::Idle, now);
      }
      else if (!targetValid) {
        this->enter(State::Lost, now);
      }
      break;
    case State::Lost:
      if (!guided) {
        this->enter(State::Idle, now);
      }
      else if (targetValid) {
        this->enter(State::Following, now);
      }
      else if (now - this->since >= this->config.lostTimeout) {
        this->enter(State::Hold, now);
      }
      break;
    case State::Hold:
      if (!guided) {
        this->enter(State::Idle, now);
      }
      break;
  }
  return this->current;
}

void Supervisor::enter(State next, TimePoint now)
{
  if (next != this->current) {
    this->current = next;
    this->since = now;
  }
}

}  // namespace follow::core
