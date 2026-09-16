#include "follow/config/ScenarioJson.h"

#include <cmath>
#include <utility>
#include <variant>

#include "ObjectReader.h"
#include "follow/config/ConfigJson.h"

namespace follow::config {

using detail::ObjectReader;
using detail::require;
using nlohmann::json;

namespace {

// Engage frame (forward, right) to NED (north, east) for a vehicle heading `yaw`.
std::pair<double, double> toNorthEast(double forward, double right, double yaw)
{
  return {std::cos(yaw) * forward - std::sin(yaw) * right, std::sin(yaw) * forward + std::cos(yaw) * right};
}

sim::TargetMotion parseMotion(const json& item, const std::string& path)
{
  require(item.is_object() && item.size() == 1, path + ": expected {\"hold\": ...}, {\"line\": ...} or {\"circle\": ...}");
  const std::string& kind = item.begin().key();
  const json& body = item.begin().value();
  const std::string bodyPath = path + "." + kind;
  if (kind == "hold") {
    ObjectReader reader(body, bodyPath, {"duration_s"});
    sim::TargetHold hold;
    reader.readRequired("duration_s", hold.durationS);
    return hold;
  }
  if (kind == "line") {
    ObjectReader reader(body, bodyPath, {"duration_s", "forward", "right"});
    sim::TargetLine line;  // velNorth/velEast hold forward/right until TargetScript::place
    reader.readRequired("duration_s", line.durationS);
    reader.read("forward", line.velNorth);
    reader.read("right", line.velEast);
    return line;
  }
  if (kind == "circle") {
    ObjectReader reader(body, bodyPath, {"duration_s", "center", "speed"});
    sim::TargetCircle circle;
    reader.readRequired("duration_s", circle.durationS);
    reader.readPoint("center", circle.centerNed);
    reader.readRequired("speed", circle.speed);
    return circle;
  }
  throw ConfigError(path + ": unknown motion '" + kind + "'");
}

std::vector<core::State> parseStates(const ObjectReader& reader, std::string_view key)
{
  std::vector<std::string> names;
  reader.read(key, names);
  std::vector<core::State> states;
  for (const std::string& name : names) {
    std::optional<core::State> state = sim::stateFromName(name);
    require(state.has_value(), reader.label(key) + ": unknown state '" + name + "'");
    states.push_back(*state);
  }
  return states;
}

}  // namespace

sim::SimTarget TargetScript::place(const sim::Pose& atEngage) const
{
  auto groundPoint = [&atEngage](const core::Vec3& engageFrame) {
    auto [north, east] = toNorthEast(engageFrame.x, engageFrame.y, atEngage.yaw);
    return core::Vec3{atEngage.positionNed.x + north, atEngage.positionNed.y + east, 0.0};
  };

  std::vector<sim::TargetMotion> motions;
  for (const sim::TargetMotion& motion : this->motions) {
    if (const auto* line = std::get_if<sim::TargetLine>(&motion)) {
      auto [north, east] = toNorthEast(line->velNorth, line->velEast, atEngage.yaw);
      motions.push_back(sim::TargetLine{.durationS = line->durationS, .velNorth = north, .velEast = east});
    }
    else if (const auto* circle = std::get_if<sim::TargetCircle>(&motion)) {
      motions.push_back(
        sim::TargetCircle{.durationS = circle->durationS, .centerNed = groundPoint(circle->centerNed), .speed = circle->speed});
    }
    else {
      motions.push_back(motion);
    }
  }
  return sim::SimTarget(groundPoint(this->start), motions, this->occlusions, this->heightM, this->widthM);
}

Scenario parseScenario(const json& doc)
{
  Scenario scenario;
  ObjectReader root(doc, "", {"name", "description", "duration_s", "target", "config", "expect"});
  root.readRequired("name", scenario.name);
  root.read("description", scenario.description);
  root.readRequired("duration_s", scenario.durationS);
  require(scenario.durationS > 0.0, "duration_s: must be positive");

  ObjectReader target = root.child("target", {"start", "height_m", "width_m", "motions", "occlusions"});
  target.readPoint("start", scenario.target.start);
  target.read("height_m", scenario.target.heightM);
  target.read("width_m", scenario.target.widthM);
  const json* motions = target.raw("motions");
  require(motions && motions->is_array() && !motions->empty(), target.label("motions") + ": expected a non-empty array");
  for (size_t i = 0; i < motions->size(); ++i) {
    scenario.target.motions.push_back(parseMotion((*motions)[i], target.label("motions") + "[" + std::to_string(i) + "]"));
  }
  std::vector<std::vector<double>> occlusions;
  target.read("occlusions", occlusions);
  for (const std::vector<double>& window : occlusions) {
    require(window.size() == 2 && window[0] < window[1], target.label("occlusions") + ": expected [start_s, end_s] with start < end");
    scenario.target.occlusions.push_back({.startS = window[0], .endS = window[1]});
  }

  if (const json* config = root.raw("config")) {
    require(config->is_object(), "config: expected an object");
    scenario.configOverrides = *config;
  }

  sim::ScenarioExpect& expect = scenario.expect;
  ObjectReader reader = root.child("expect",
                                   {"states",
                                    "states_in_order",
                                    "final_state",
                                    "bearing_max_deg",
                                    "bearing_settle_s",
                                    "lock_distance_tolerance",
                                    "final_distance",
                                    "min_distance_m",
                                    "lag",
                                    "no_forward_speed"});
  if (reader.raw("states")) {
    expect.states = parseStates(reader, "states");
  }
  expect.statesInOrder = parseStates(reader, "states_in_order");
  if (reader.raw("final_state")) {
    std::string name;
    reader.read("final_state", name);
    expect.finalState = sim::stateFromName(name);
    require(expect.finalState.has_value(), reader.label("final_state") + ": unknown state '" + name + "'");
  }
  reader.readOptional("bearing_max_deg", expect.bearingMaxDeg);
  reader.read("bearing_settle_s", expect.bearingSettleS);
  reader.readOptional("lock_distance_tolerance", expect.lockDistanceTolerance);
  if (reader.raw("final_distance")) {
    ObjectReader finalDistance = reader.child("final_distance", {"reference", "tolerance"});
    std::string reference = "lock";
    finalDistance.read("reference", reference);
    require(reference == "lock" || reference == "set", finalDistance.label("reference") + ": expected \"lock\" or \"set\"");
    expect.finalDistanceReference =
      reference == "lock" ? sim::ScenarioExpect::DistanceReference::Lock : sim::ScenarioExpect::DistanceReference::Set;
    double tolerance = 0.0;
    finalDistance.readRequired("tolerance", tolerance);
    expect.finalDistanceTolerance = tolerance;
  }
  reader.readOptional("min_distance_m", expect.minDistanceM);
  if (reader.raw("lag")) {
    ObjectReader lagReader = reader.child("lag", {"from_s", "to_s", "max_over_lock_m", "max_spread_m"});
    sim::ScenarioExpect::Lag lag;
    lagReader.readRequired("from_s", lag.fromS);
    lagReader.readRequired("to_s", lag.toS);
    lagReader.readRequired("max_over_lock_m", lag.maxOverLockM);
    lagReader.readRequired("max_spread_m", lag.maxSpreadM);
    expect.lag = lag;
  }
  reader.read("no_forward_speed", expect.noForwardSpeed);
  return scenario;
}

Scenario loadScenario(const std::filesystem::path& path)
{
  json doc = readJsonFile(path);
  try {
    return parseScenario(doc);
  }
  catch (const ConfigError& e) {
    throw ConfigError(path.string() + ": " + e.what());
  }
}

}  // namespace follow::config
