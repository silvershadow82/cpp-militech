#pragma once

#include <cstddef>
#include <map>
#include <optional>
#include <set>
#include <vector>

#include "mission_control/grid.hpp"

namespace mission_control {

struct ObservedCell {
  Cell cell;
  CellState state = CellState::Unknown;
  int contactId = 0;
};

struct Observation {
  Cell robot;
  std::vector<ObservedCell> cells;
};

class MapMemory {
public:
  void update(const Observation& observation);

  [[nodiscard]] CellState state(Cell cell) const;
  [[nodiscard]] bool isKnown(Cell cell) const;
  [[nodiscard]] bool isVisited(Cell cell) const;

  [[nodiscard]] bool isFrontier(Cell cell) const;
  [[nodiscard]] std::vector<Cell> frontiers() const;

  [[nodiscard]] std::optional<Direction> firstStepToward(Cell from, const std::vector<Cell>& targets) const;

  [[nodiscard]] std::size_t knownCellCount() const;
  [[nodiscard]] std::size_t visitedCellCount() const;

private:
  std::map<Cell, CellState> cells;
  std::set<Cell> visited;
};

}  // namespace mission_control
