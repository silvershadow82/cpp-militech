#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string_view>

namespace mission_control {

struct Cell {
  int x = 0;
  int y = 0;
};

bool operator==(Cell left, Cell right);
bool operator!=(Cell left, Cell right);
bool operator<(Cell left, Cell right);

enum class Direction : std::uint8_t {
  Up = 0,
  Down = 1,
  Left = 2,
  Right = 3,
};

enum class CellState : std::uint8_t {
  Unknown,
  Wall,
  Free,
  Start,
  Contact,
  Processed,
};

inline constexpr std::array<Direction, 4> allDirections{
  Direction::Up,
  Direction::Down,
  Direction::Left,
  Direction::Right,
};

[[nodiscard]] Cell step(Cell from, Direction direction);

[[nodiscard]] std::optional<Direction> directionBetween(Cell from, Cell to);

[[nodiscard]] bool isWalkable(CellState state);

[[nodiscard]] CellState cellStateFromType(std::string_view cell_type);

[[nodiscard]] std::string_view to_string(Direction direction);
[[nodiscard]] std::string_view to_string(CellState state);

}  // namespace mission_control
