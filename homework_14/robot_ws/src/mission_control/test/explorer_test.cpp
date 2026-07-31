#include <string>
#include <string_view>
#include <vector>

#include "gtest/gtest.h"

#include "mission_control/explorer.hpp"

namespace mission_control {
namespace {

using Grid = std::vector<std::string>;

Observation scanAt(const Grid& grid, const Cell robot, const int contact_id = 0)
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
      if (observed.state == CellState::Contact || observed.state == CellState::Processed) {
        observed.contactId = contact_id;
      }
      observation.cells.push_back(observed);
    }
  }

  return observation;
}

TEST(ExplorerTest, WaitsBeforeFirstObservation)
{
  const Explorer explorer;

  EXPECT_EQ(explorer.decide().kind, Decision::Kind::Wait);
  EXPECT_FALSE(explorer.explorationComplete());
}

TEST(ExplorerTest, MovesTowardNearestFrontier)
{
  const Grid grid{
    "#####",
    "#S..#",
    "#####",
  };
  Explorer explorer;

  explorer.observe(scanAt(grid, Cell{1, 1}));
  const auto decision = explorer.decide();

  EXPECT_EQ(decision.kind, Decision::Kind::Move);
  EXPECT_EQ(decision.direction, Direction::Right);
}

TEST(ExplorerTest, ReportsDoneWhenNoFrontiersRemain)
{
  const Grid grid{
    "####",
    "#S.#",
    "####",
  };
  Explorer explorer;

  explorer.observe(scanAt(grid, Cell{1, 1}));
  explorer.observe(scanAt(grid, Cell{2, 1}));

  EXPECT_EQ(explorer.decide().kind, Decision::Kind::Done);
  EXPECT_TRUE(explorer.explorationComplete());
}

TEST(ExplorerTest, EngagesVisibleContactBeforeMoving)
{
  const Grid grid{
    "#####",
    "#S.C#",
    "#####",
  };
  Explorer explorer;

  explorer.observe(scanAt(grid, Cell{2, 1}, 7));
  const auto decision = explorer.decide();

  ASSERT_EQ(decision.kind, Decision::Kind::Engage);
  EXPECT_EQ(decision.contact.contact_id, 7);
  EXPECT_EQ(decision.contact.cell, (Cell{3, 1}));
}

TEST(ExplorerTest, EngagesDiagonallyVisibleContact)
{
  const Grid grid{
    "#####",
    "#S..#",
    "#.#C#",
    "#####",
  };
  Explorer explorer;

  explorer.observe(scanAt(grid, Cell{2, 1}, 4));
  const auto decision = explorer.decide();

  ASSERT_EQ(decision.kind, Decision::Kind::Engage);
  EXPECT_EQ(decision.contact.cell, (Cell{3, 2}));
}

TEST(ExplorerTest, WaitsInsteadOfMovingWhileContactIsEngaged)
{
  const Grid grid{
    "#####",
    "#S.C#",
    "#####",
  };
  Explorer explorer;

  explorer.observe(scanAt(grid, Cell{2, 1}, 7));
  explorer.markEngaged(7);

  EXPECT_EQ(explorer.decide().kind, Decision::Kind::Wait);
  EXPECT_TRUE(explorer.isEngaged(7));
}

TEST(ExplorerTest, ResumesExplorationAfterContactIsProcessed)
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
  Explorer explorer;

  explorer.observe(scanAt(active, Cell{1, 1}, 7));
  explorer.observe(scanAt(active, Cell{2, 1}, 7));
  explorer.markEngaged(7);
  ASSERT_EQ(explorer.decide().kind, Decision::Kind::Wait);

  explorer.observe(scanAt(processed, Cell{2, 1}, 7));

  EXPECT_FALSE(explorer.isEngaged(7));
  const auto decision = explorer.decide();
  EXPECT_EQ(decision.kind, Decision::Kind::Move);
  EXPECT_EQ(decision.direction, Direction::Right);
}

TEST(ExplorerTest, EquidistantFrontiersAreResolvedByDirectionOrder)
{
  const Grid grid{
    "#####",
    "#...#",
    "#####",
  };
  Explorer explorer;

  explorer.observe(scanAt(grid, Cell{2, 1}));

  const auto decision = explorer.decide();

  ASSERT_EQ(decision.kind, Decision::Kind::Move);
  EXPECT_EQ(decision.direction, Direction::Left);
}

TEST(ExplorerTest, EngagesLowestContactIdFirst)
{
  Observation observation;
  observation.robot = Cell{1, 1};
  observation.cells = {
    ObservedCell{Cell{1, 1}, CellState::Start, 0},
    ObservedCell{Cell{2, 1}, CellState::Contact, 5},
    ObservedCell{Cell{1, 2}, CellState::Contact, 2},
  };
  Explorer explorer;

  explorer.observe(observation);
  const auto first = explorer.decide();
  ASSERT_EQ(first.kind, Decision::Kind::Engage);
  EXPECT_EQ(first.contact.contact_id, 2);

  explorer.markEngaged(2);
  const auto second = explorer.decide();
  ASSERT_EQ(second.kind, Decision::Kind::Engage);
  EXPECT_EQ(second.contact.contact_id, 5);

  explorer.markEngaged(5);
  EXPECT_EQ(explorer.decide().kind, Decision::Kind::Wait);
}

TEST(ExplorerTest, ReportsFailedWhenFrontierIsUnreachable)
{
  Observation observation;
  observation.robot = Cell{1, 1};
  observation.cells = {
    ObservedCell{Cell{0, 0}, CellState::Wall, 0},
    ObservedCell{Cell{1, 0}, CellState::Wall, 0},
    ObservedCell{Cell{2, 0}, CellState::Wall, 0},
    ObservedCell{Cell{0, 1}, CellState::Wall, 0},
    ObservedCell{Cell{1, 1}, CellState::Start, 0},
    ObservedCell{Cell{2, 1}, CellState::Wall, 0},
    ObservedCell{Cell{0, 2}, CellState::Wall, 0},
    ObservedCell{Cell{1, 2}, CellState::Wall, 0},
    ObservedCell{Cell{2, 2}, CellState::Wall, 0},
    ObservedCell{Cell{9, 9}, CellState::Free, 0},
  };
  Explorer explorer;

  explorer.observe(observation);

  EXPECT_EQ(explorer.decide().kind, Decision::Kind::Failed);
  EXPECT_FALSE(explorer.explorationComplete());
}

TEST(ExplorerTest, ForgetEngagedAllowsRetry)
{
  const Grid grid{
    "#####",
    "#S.C#",
    "#####",
  };
  Explorer explorer;

  explorer.observe(scanAt(grid, Cell{2, 1}, 7));
  explorer.markEngaged(7);
  ASSERT_EQ(explorer.decide().kind, Decision::Kind::Wait);

  explorer.forgetEngaged(7);

  EXPECT_EQ(explorer.decide().kind, Decision::Kind::Engage);
}

}  // namespace
}  // namespace mission_control
