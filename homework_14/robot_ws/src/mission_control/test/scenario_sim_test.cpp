#include <cstdint>
#include <filesystem>
#include <string>

#include "gtest/gtest.h"

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "underground_world/scenario_loader.hpp"
#include "underground_world/world_model.hpp"

#include "mission_control/explorer.hpp"

namespace mission_control {
namespace {

constexpr std::uint32_t iterationLimit = 2000;

std::filesystem::path scenarioPath(const std::string& filename)
{
  return std::filesystem::path(ament_index_cpp::get_package_share_directory("underground_world")) / "config" / filename;
}

Observation toObservation(const underground_world::LocalScanData& scan)
{
  Observation observation;
  observation.robot = Cell{scan.robot.x, scan.robot.y};
  observation.cells.reserve(scan.cells.size());

  for (const auto& cell : scan.cells) {
    ObservedCell observed;
    observed.cell = Cell{cell.position.x, cell.position.y};
    observed.state = cellStateFromType(underground_world::cell_kind_to_string(cell.kind));
    observed.contactId = cell.contact_id;
    observation.cells.push_back(observed);
  }

  return observation;
}

struct SimOutcome {
  underground_world::MetricsSnapshot metrics;
  underground_world::ResultSnapshot result;
  Decision::Kind last_decision = Decision::Kind::Wait;
  std::uint32_t iterations = 0;
};

SimOutcome runScenario(const std::string& filename)
{
  underground_world::WorldModel world(underground_world::load_scenario(scenarioPath(filename)));
  Explorer explorer;
  SimOutcome outcome;

  for (; outcome.iterations < iterationLimit; ++outcome.iterations) {
    const auto snapshot = world.snapshot();
    explorer.observe(toObservation(snapshot.scan));

    if (world.terminal()) {
      break;
    }

    const auto decision = explorer.decide();
    outcome.last_decision = decision.kind;

    if (decision.kind == Decision::Kind::Engage) {
      explorer.markEngaged(decision.contact.contact_id);
      world.apply_enemy_down(decision.contact.contact_id, underground_world::Position{decision.contact.cell.x, decision.contact.cell.y});
      continue;
    }

    if (decision.kind == Decision::Kind::Move) {
      world.apply_move(static_cast<std::uint8_t>(decision.direction));
      continue;
    }

    break;
  }

  outcome.metrics = world.metrics();
  outcome.result = world.result();
  return outcome;
}

void expectScenarioPasses(const std::string& filename, const std::uint32_t expected_steps)
{
  const auto outcome = runScenario(filename);

  SCOPED_TRACE(filename);
  EXPECT_LT(outcome.iterations, kIterationLimit);

  EXPECT_EQ(outcome.result.mission_result, "SUCCESS");
  EXPECT_FLOAT_EQ(outcome.metrics.map_coverage_percent, 100.0F);
  EXPECT_EQ(outcome.metrics.contacts_seen, outcome.metrics.contacts_down);
  EXPECT_EQ(outcome.metrics.invalid_moves, 0U);
  EXPECT_EQ(outcome.metrics.invalid_triggers, 0U);
  EXPECT_EQ(outcome.metrics.duplicate_triggers, 0U);
  EXPECT_LT(outcome.metrics.steps_taken, outcome.result.max_steps);

  EXPECT_EQ(outcome.metrics.steps_taken, expected_steps);
}

TEST(ScenarioSimTest, TrainingCorridor)
{
  expectScenarioPasses("training_corridor.yaml", 5);
}

TEST(ScenarioSimTest, SmallRooms)
{
  expectScenarioPasses("small_rooms.yaml", 37);
}

TEST(ScenarioSimTest, BranchingTrench)
{
  expectScenarioPasses("branching_trench.yaml", 65);
}

TEST(ScenarioSimTest, DeadEndBunker)
{
  expectScenarioPasses("dead_end_bunker.yaml", 95);
}

}  // namespace
}  // namespace mission_control
