#pragma once

#include <cstdint>
#include <optional>

#include "follow/core/Config.h"
#include "follow/core/Types.h"

namespace follow::core {

// Follow-mode state machine driven by the flight mode, FC heartbeat and target validity.
class Supervisor {
public:
  explicit Supervisor(const SupervisorConfig& config)
    : config(config)
  {
  }

  State update(TimePoint now, std::optional<TimePoint> lastHeartbeat, uint32_t customMode, bool targetValid);
  State state() const { return this->current; }

private:
  void enter(State next, TimePoint now);

  SupervisorConfig config;
  State current{State::Idle};
  TimePoint since{};
  // Unknown after start-up or a heartbeat loss, so an FC already in GUIDED never engages by itself.
  std::optional<uint32_t> previousMode{};
};

}  // namespace follow::core
