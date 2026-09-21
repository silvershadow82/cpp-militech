#pragma once

#include <cstdint>
#include <optional>

#include "Types.h"
#include "models/Config.h"

namespace follow::control {

// Follow-mode state machine driven by the flight mode, FC heartbeat and target validity.
class Supervisor {
public:
  explicit Supervisor(const models::SupervisorConfig& config)
    : config(config)
  {
  }

  models::State update(models::TimePoint now, std::optional<models::TimePoint> lastHeartbeat, uint32_t customMode, bool targetValid);
  models::State state() const { return this->current; }

private:
  void enter(models::State next, models::TimePoint now);

  models::SupervisorConfig config;
  models::State current{models::State::Idle};
  models::TimePoint since{};
  // Unknown after start-up or a heartbeat loss, so an FC already in GUIDED never engages by itself.
  std::optional<uint32_t> previousMode{};
};

}  // namespace follow::control
