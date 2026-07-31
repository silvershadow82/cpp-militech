#include "mission_control/grid.hpp"

#include <tuple>

namespace mission_control {

bool operator==(const Cell left, const Cell right)
{
  return left.x == right.x && left.y == right.y;
}

bool operator!=(const Cell left, const Cell right)
{
  return !(left == right);
}

bool operator<(const Cell left, const Cell right)
{
  return std::tie(left.y, left.x) < std::tie(right.y, right.x);
}

Cell step(const Cell from, const Direction direction)
{
  switch (direction) {
    case Direction::Up:
      return Cell{from.x, from.y - 1};
    case Direction::Down:
      return Cell{from.x, from.y + 1};
    case Direction::Left:
      return Cell{from.x - 1, from.y};
    case Direction::Right:
      return Cell{from.x + 1, from.y};
  }

  return from;
}

std::optional<Direction> directionBetween(const Cell from, const Cell to)
{
  for (const auto direction : allDirections) {
    if (step(from, direction) == to) {
      return direction;
    }
  }

  return std::nullopt;
}

bool isWalkable(const CellState state)
{
  return state == CellState::Free || state == CellState::Start || state == CellState::Processed;
}

CellState cellStateFromType(const std::string_view cellType)
{
  if (cellType == "#") {
    return CellState::Wall;
  }
  if (cellType == ".") {
    return CellState::Free;
  }
  if (cellType == "S") {
    return CellState::Start;
  }
  if (cellType == "C") {
    return CellState::Contact;
  }
  if (cellType == "x") {
    return CellState::Processed;
  }

  return CellState::Unknown;
}

std::string_view to_string(const Direction direction)
{
  switch (direction) {
    case Direction::Up:
      return "UP";
    case Direction::Down:
      return "DOWN";
    case Direction::Left:
      return "LEFT";
    case Direction::Right:
      return "RIGHT";
  }

  return "UNKNOWN";
}

std::string_view to_string(const CellState state)
{
  switch (state) {
    case CellState::Unknown:
      return "?";
    case CellState::Wall:
      return "#";
    case CellState::Free:
      return ".";
    case CellState::Start:
      return "S";
    case CellState::Contact:
      return "C";
    case CellState::Processed:
      return "x";
  }

  return "?";
}

}  // namespace mission_control
