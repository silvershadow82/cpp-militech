#include <string>
#include <string_view>
#include <vector>

#include "gtest/gtest.h"

#include "mission_control/map_memory.hpp"

namespace mission_control {
namespace {

using Grid = std::vector<std::string>;

// Локальний огляд 3x3 навколо робота, як його публікує underground_world_node:
// клітинки за межами сітки просто не потрапляють у масив.
Observation scanAt(const Grid& grid, const Cell robot)
{
  Observation observation;
  observation.robot = robot;

  const auto height = static_cast<int>(grid.size());
  const auto width = static_cast<int>(grid.front().size());

  for (int y = robot.y - 1; y <= robot.y + 1; ++y) {
    for (int x = robot.x - 1; x <= robot.x + 1; ++x) {
      if (y < 0 || x < 0 || y >= height || x >= width) {
        continue;
      }

      ObservedCell observed;
      observed.cell = Cell{x, y};
      observed.state = cellStateFromType(std::string_view(&grid.at(static_cast<std::size_t>(y)).at(static_cast<std::size_t>(x)), 1));
      observation.cells.push_back(observed);
    }
  }

  return observation;
}

TEST(MapMemoryTest, EverythingIsUnknownBeforeAnyObservation)
{
  const MapMemory memory;

  EXPECT_EQ(memory.state(Cell{0, 0}), CellState::Unknown);
  EXPECT_FALSE(memory.isKnown(Cell{0, 0}));
  EXPECT_FALSE(memory.isVisited(Cell{0, 0}));
  EXPECT_EQ(memory.knownCellCount(), 0U);
  EXPECT_TRUE(memory.frontiers().empty());
}

TEST(MapMemoryTest, UpdateStoresObservedStatesAndMarksRobotCellVisited)
{
  const Grid grid{
    "#####",
    "#S..#",
    "#####",
  };
  MapMemory memory;

  memory.update(scanAt(grid, Cell{1, 1}));

  EXPECT_EQ(memory.state(Cell{1, 1}), CellState::Start);
  EXPECT_EQ(memory.state(Cell{2, 1}), CellState::Free);
  EXPECT_EQ(memory.state(Cell{0, 0}), CellState::Wall);
  EXPECT_EQ(memory.state(Cell{3, 1}), CellState::Unknown);

  EXPECT_TRUE(memory.isVisited(Cell{1, 1}));
  EXPECT_FALSE(memory.isVisited(Cell{2, 1}));
  EXPECT_EQ(memory.visitedCellCount(), 1U);
}

TEST(MapMemoryTest, FrontierIsWalkableUnvisitedCellWithUnknownNeighbour)
{
  const Grid grid{
    "#####",
    "#S..#",
    "#####",
  };
  MapMemory memory;

  memory.update(scanAt(grid, Cell{1, 1}));

  EXPECT_TRUE(memory.isFrontier(Cell{2, 1}));

  const auto frontiers = memory.frontiers();
  ASSERT_EQ(frontiers.size(), 1U);
  EXPECT_EQ(frontiers.front(), (Cell{2, 1}));
}

TEST(MapMemoryTest, VisitedCellIsNeverFrontier)
{
  const Grid grid{
    "#####",
    "#S..#",
    "#####",
  };
  MapMemory memory;

  memory.update(scanAt(grid, Cell{1, 1}));
  ASSERT_TRUE(memory.isFrontier(Cell{2, 1}));

  memory.update(scanAt(grid, Cell{2, 1}));

  EXPECT_FALSE(memory.isFrontier(Cell{2, 1}));
  EXPECT_TRUE(memory.isFrontier(Cell{3, 1}));
}

TEST(MapMemoryTest, WallsAndUnknownCellsAreNotFrontiers)
{
  const Grid grid{
    "#####",
    "#S..#",
    "#####",
  };
  MapMemory memory;

  memory.update(scanAt(grid, Cell{1, 1}));

  EXPECT_FALSE(memory.isFrontier(Cell{0, 0}));
  EXPECT_FALSE(memory.isFrontier(Cell{4, 1}));
}

TEST(MapMemoryTest, ExploredCorridorLeavesNoFrontiers)
{
  const Grid grid{
    "####",
    "#S.#",
    "####",
  };
  MapMemory memory;

  memory.update(scanAt(grid, Cell{1, 1}));
  memory.update(scanAt(grid, Cell{2, 1}));

  EXPECT_TRUE(memory.frontiers().empty());
}

TEST(MapMemoryTest, ProcessedContactBecomesWalkable)
{
  const Grid active{
    "#####",
    "#S.C#",
    "#####",
  };
  const Grid processed{
    "#####",
    "#S.x#",
    "#####",
  };
  MapMemory memory;

  memory.update(scanAt(active, Cell{2, 1}));
  EXPECT_EQ(memory.state(Cell{3, 1}), CellState::Contact);
  EXPECT_FALSE(memory.firstStepToward(Cell{1, 1}, {Cell{3, 1}}).has_value());

  memory.update(scanAt(processed, Cell{2, 1}));
  EXPECT_EQ(memory.state(Cell{3, 1}), CellState::Processed);
  EXPECT_EQ(memory.firstStepToward(Cell{1, 1}, {Cell{3, 1}}), Direction::Right);
}

TEST(MapMemoryTest, FirstStepTowardFollowsShortestKnownPath)
{
  const Grid grid{
    "#####",
    "#S..#",
    "#.###",
    "#...#",
    "#####",
  };
  MapMemory memory;

  for (const auto cell : {Cell{1, 1}, Cell{2, 1}, Cell{3, 1}, Cell{1, 2}, Cell{1, 3}, Cell{2, 3}, Cell{3, 3}}) {
    memory.update(scanAt(grid, cell));
  }

  EXPECT_EQ(memory.firstStepToward(Cell{1, 1}, {Cell{3, 1}}), Direction::Right);
  EXPECT_EQ(memory.firstStepToward(Cell{1, 1}, {Cell{1, 3}}), Direction::Down);
  EXPECT_EQ(memory.firstStepToward(Cell{1, 3}, {Cell{1, 1}}), Direction::Up);
}

TEST(MapMemoryTest, FirstStepTowardPicksNearestTarget)
{
  const Grid grid{
    "#######",
    "#.....#",
    "#######",
  };
  MapMemory memory;

  for (int x = 1; x <= 5; ++x) {
    memory.update(scanAt(grid, Cell{x, 1}));
  }

  EXPECT_EQ(memory.firstStepToward(Cell{3, 1}, {Cell{1, 1}, Cell{4, 1}}), Direction::Right);
  EXPECT_EQ(memory.firstStepToward(Cell{3, 1}, {Cell{2, 1}, Cell{5, 1}}), Direction::Left);
}

TEST(MapMemoryTest, FirstStepTowardReturnsNulloptWhenTargetIsUnreachable)
{
  const Grid grid{
    "#####",
    "#S#.#",
    "#####",
  };
  MapMemory memory;

  memory.update(scanAt(grid, Cell{1, 1}));

  EXPECT_FALSE(memory.firstStepToward(Cell{1, 1}, {Cell{3, 1}}).has_value());
  EXPECT_FALSE(memory.firstStepToward(Cell{1, 1}, {}).has_value());
}

}  // namespace
}  // namespace mission_control
