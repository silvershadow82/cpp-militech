#pragma once

#include <cstdint>
#include <istream>
#include <optional>
#include <ostream>
#include <vector>

#include "Types.h"
#include "sim/ScenarioRunner.h"

namespace follow::util {

// One control step as written to the CSV run log.
struct LogRow {
  double tS{0.0};
  models::State state{models::State::Idle};
  uint32_t customMode{0};
  models::TargetState target{};
  std::optional<models::VelocityCmd> setpoint{};
  std::optional<double> trueBearingDeg{};  // simulation only
  std::optional<double> trueDistanceM{};   // simulation only
};

// CSV columns: t,state,mode,valid,bearing_deg,ratio,distance_m,source,vx,yaw_rate,true_bearing_deg,true_distance_m.
// Absent values are empty fields; yaw_rate is rad/s.
class RunLogWriter {
public:
  explicit RunLogWriter(std::ostream& out);
  void write(const LogRow& row);

private:
  std::ostream& out;
};

// Reads a simulation run log back into scenario steps for sim::checkRun.
// Throws std::runtime_error naming the line on a malformed row or a row without ground truth.
std::vector<sim::StepRecord> readRunLog(std::istream& in);

}  // namespace follow::util
