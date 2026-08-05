#include <algorithm>
#include <set>
#include <vector>

#include "gtest/gtest.h"

#include "mission_control/grid.hpp"

namespace mission_control {
namespace {

TEST(GridTest, StepMatchesMoveCommandSemantics)
{
  const Cell origin{2, 3};

  EXPECT_EQ(step(origin, Direction::Up), (Cell{2, 2}));
  EXPECT_EQ(step(origin, Direction::Down), (Cell{2, 4}));
  EXPECT_EQ(step(origin, Direction::Left), (Cell{1, 3}));
  EXPECT_EQ(step(origin, Direction::Right), (Cell{3, 3}));
}

TEST(GridTest, DirectionValuesMatchMoveCommandConstants)
{
  EXPECT_EQ(static_cast<std::uint8_t>(Direction::Up), 0U);
  EXPECT_EQ(static_cast<std::uint8_t>(Direction::Down), 1U);
  EXPECT_EQ(static_cast<std::uint8_t>(Direction::Left), 2U);
  EXPECT_EQ(static_cast<std::uint8_t>(Direction::Right), 3U);
}

TEST(GridTest, DirectionBetweenFindsAdjacentCells)
{
  const Cell origin{4, 4};

  for (const auto direction : allDirections) {
    const auto found = directionBetween(origin, step(origin, direction));
    ASSERT_TRUE(found.has_value());
    EXPECT_EQ(*found, direction);
  }
}

TEST(GridTest, DirectionBetweenRejectsNonAdjacentCells)
{
  const Cell origin{4, 4};

  EXPECT_FALSE(directionBetween(origin, origin).has_value());
  EXPECT_FALSE(directionBetween(origin, Cell{5, 5}).has_value());  // діагональ
  EXPECT_FALSE(directionBetween(origin, Cell{4, 6}).has_value());  // відстань 2
}

TEST(GridTest, ActiveContactIsNotWalkable)
{
  EXPECT_TRUE(isWalkable(CellState::Free));
  EXPECT_TRUE(isWalkable(CellState::Start));
  EXPECT_TRUE(isWalkable(CellState::Processed));

  EXPECT_FALSE(isWalkable(CellState::Contact));
  EXPECT_FALSE(isWalkable(CellState::Wall));
  EXPECT_FALSE(isWalkable(CellState::Unknown));
}

TEST(GridTest, CellStateFromTypeCoversScanAlphabet)
{
  EXPECT_EQ(cellStateFromType("#"), CellState::Wall);
  EXPECT_EQ(cellStateFromType("."), CellState::Free);
  EXPECT_EQ(cellStateFromType("S"), CellState::Start);
  EXPECT_EQ(cellStateFromType("C"), CellState::Contact);
  EXPECT_EQ(cellStateFromType("x"), CellState::Processed);

  EXPECT_EQ(cellStateFromType(""), CellState::Unknown);
  EXPECT_EQ(cellStateFromType("E"), CellState::Unknown);
}

TEST(GridTest, CellStateRoundTripsThroughScanAlphabet)
{
  for (const auto state : {CellState::Wall, CellState::Free, CellState::Start, CellState::Contact, CellState::Processed}) {
    EXPECT_EQ(cellStateFromType(to_string(state)), state);
  }
}

TEST(GridTest, CellOrderingIsRowMajor)
{
  const std::set<Cell> cells{Cell{2, 1}, Cell{0, 1}, Cell{1, 0}};
  const std::vector<Cell> expected{Cell{1, 0}, Cell{0, 1}, Cell{2, 1}};

  EXPECT_TRUE(std::equal(cells.begin(), cells.end(), expected.begin(), expected.end()));
}

}  // namespace
}  // namespace mission_control
