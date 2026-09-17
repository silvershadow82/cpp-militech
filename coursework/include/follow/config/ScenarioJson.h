#pragma once

#include <filesystem>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

#include "follow/core/Types.h"
#include "follow/sim/ScenarioCheck.h"
#include "follow/sim/SimTarget.h"
#include "follow/sim/SyntheticCamera.h"

namespace follow::config {

// Target script in the engage frame: x forward and y right of the vehicle at the moment the pilot
// engages, origin under the vehicle. Line velocities and circle centers use the same frame.
struct TargetScript {
  core::Vec3 start{};
  std::vector<sim::TargetMotion> motions{};
  std::vector<sim::OcclusionWindow> occlusions{};  // seconds after engage
  double heightM{1.7};
  double widthM{0.5};

  // The script in NED for a vehicle at `atEngage`: rotated by its yaw, moved under its position.
  sim::SimTarget place(const sim::Pose& atEngage) const;
};

struct Scenario {
  std::string name;
  std::string description;
  double durationS{20.0};  // after engage
  TargetScript target{};
  nlohmann::json configOverrides = nlohmann::json::object();  // JSON merge patch over follow.json
  sim::ScenarioExpect expect{};
};

// Throws ConfigError naming the offending key, e.g. "expect.final_state".
Scenario parseScenario(const nlohmann::json& doc);
Scenario loadScenario(const std::filesystem::path& path);

}  // namespace follow::config
