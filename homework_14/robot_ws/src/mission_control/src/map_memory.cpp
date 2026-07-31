#include "mission_control/map_memory.hpp"

#include <queue>

namespace mission_control {

void MapMemory::update(const Observation& observation)
{
  this->visited.insert(observation.robot);

  for (const auto& observed : observation.cells) {
    this->cells[observed.cell] = observed.state;
  }
}

CellState MapMemory::state(const Cell cell) const
{
  const auto iter = this->cells.find(cell);
  return iter == this->cells.end() ? CellState::Unknown : iter->second;
}

bool MapMemory::isKnown(const Cell cell) const
{
  return this->cells.find(cell) != this->cells.end();
}

bool MapMemory::isVisited(const Cell cell) const
{
  return this->cells.find(cell) != this->cells.end();
}

bool MapMemory::isFrontier(const Cell cell) const
{
  if (!isWalkable(state(cell)) || isVisited(cell)) {
    return false;
  }

  for (int dy = -1; dy <= 1; ++dy) {
    for (int dx = -1; dx <= 1; ++dx) {
      if (!isKnown(Cell{cell.x + dx, cell.y + dy})) {
        return true;
      }
    }
  }

  return false;
}

std::vector<Cell> MapMemory::frontiers() const
{
  std::vector<Cell> found;
  for (const auto& [cell, state] : this->cells) {
    if (isFrontier(cell)) {
      found.push_back(cell);
    }
  }
  return found;
}

std::optional<Direction> MapMemory::firstStepToward(const Cell from, const std::vector<Cell>& targets) const
{
  if (targets.empty()) {
    return std::nullopt;
  }

  const std::set<Cell> wanted(targets.begin(), targets.end());
  std::map<Cell, Direction> firstStep;
  std::set<Cell> reached{from};
  std::queue<Cell> pending;
  pending.push(from);

  while (!pending.empty()) {
    const auto current = pending.front();
    pending.pop();

    for (const auto direction : allDirections) {
      const auto next = step(current, direction);
      if (reached.find(next) != reached.end() || !isWalkable(state(next))) {
        continue;
      }

      const auto entry = current == from ? direction : firstStep.at(current);
      reached.insert(next);
      firstStep.emplace(next, entry);

      if (wanted.find(next) != wanted.end()) {
        return entry;
      }

      pending.push(next);
    }
  }

  return std::nullopt;
}

std::size_t MapMemory::knownCellCount() const
{
  return this->cells.size();
}

std::size_t MapMemory::visitedCellCount() const
{
  return this->cells.size();
}

}  // namespace mission_control
