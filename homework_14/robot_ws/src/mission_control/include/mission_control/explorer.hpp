#pragma once

#include <cstdint>
#include <optional>
#include <set>
#include <vector>

#include "mission_control/map_memory.hpp"

namespace mission_control {

struct VisibleContact {
  int contactId = 0;
  Cell cell;
};

struct Decision {
  enum class Kind : std::uint8_t {
    Wait,
    Move,
    Engage,
    Done,
    Failed,
  };

  Kind kind = Kind::Wait;
  Direction direction = Direction::Up;
  VisibleContact contact;
};

class Explorer {
public:
  void observe(const Observation& observation);
  [[nodiscard]] Decision decide() const;

  void markEngaged(int contactId);
  void forgetEngaged(int contactId);

  [[nodiscard]] bool isEngaged(int contactId) const;

  [[nodiscard]] const MapMemory& mapMemory() const;
  [[nodiscard]] bool explorationComplete() const;

private:
  [[nodiscard]] std::optional<VisibleContact> pendingContact() const;

  MapMemory memory;
  Cell robot;
  std::vector<VisibleContact> visibleContacts;
  std::set<int> engaged;
  bool observed = false;
};

}  // namespace mission_control
