#include "StatCollector.h"
#include "models/Angles.h"
#include "sim/ScenarioCheck.h"

#include <cstdio>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>

namespace follow::util {

namespace {

constexpr const char *kHeader = "t,state,mode,valid,bearing_deg,ratio,distance_m,source,vx,yaw_rate,true_bearing_deg,true_distance_m";

std::string fixed(double value, int decimals)
{
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.*f", decimals, value);
  return buffer;
}

std::string optionalFixed(const std::optional<double> &value, int decimals)
{
  return value ? fixed(*value, decimals) : std::string();
}

const char *sourceName(models::DistanceSource source)
{
  switch (source) {
    case models::DistanceSource::Relative:
      return "Relative";
    case models::DistanceSource::KnownSize:
      return "KnownSize";
    case models::DistanceSource::Range:
      return "Range";
  }
  return "?";
}

std::vector<std::string> splitFields(const std::string &line)
{
  std::vector<std::string> fields;
  std::string field;
  std::istringstream stream(line);
  while (std::getline(stream, field, ',')) {
    fields.push_back(field);
  }
  if (!line.empty() && line.back() == ',') {
    fields.emplace_back();
  }
  return fields;
}

double toDouble(const std::string &text, size_t lineNumber, const char *column)
{
  try {
    size_t used = 0;
    double value = std::stod(text, &used);
    if (used == text.size()) {
      return value;
    }
  }
  catch (const std::exception &) {
  }
  throw std::runtime_error("run log line " + std::to_string(lineNumber) + ": bad " + column + " '" + text + "'");
}

}  // namespace

StatCollector::StatCollector(std::ostream &out)
  : out(out)
{
  this->out << kHeader << '\n';
}

void StatCollector::write(const LogRow &row)
{
  const models::TargetState &target = row.target;
  this->out << fixed(row.tS, 3) << ',' << sim::stateName(row.state) << ',' << row.customMode << ',' << (target.valid ? 1 : 0) << ','
            << (target.valid ? fixed(models::radToDeg(target.bearingRad), 2) : "") << ',' << (target.valid ? fixed(target.ratio, 3) : "")
            << ',' << optionalFixed(target.distanceM, 2) << ',' << (target.valid ? sourceName(target.source) : "") << ','
            << (row.setpoint ? fixed(row.setpoint->vx, 6) : "") << ',' << (row.setpoint ? fixed(row.setpoint->yawRate, 6) : "") << ','
            << optionalFixed(row.trueBearingDeg, 2) << ',' << optionalFixed(row.trueDistanceM, 3) << '\n';
  this->out.flush();
}

std::vector<sim::StepRecord> readRunLog(std::istream &in)
{
  std::string line;
  if (!std::getline(in, line) || line != kHeader) {
    throw std::runtime_error("run log: missing header '" + std::string(kHeader) + "'");
  }
  std::vector<sim::StepRecord> steps;
  for (size_t lineNumber = 2; std::getline(in, line); ++lineNumber) {
    if (line.empty()) {
      continue;
    }
    std::vector<std::string> f = splitFields(line);
    if (f.size() != 12) {
      throw std::runtime_error("run log line " + std::to_string(lineNumber) + ": expected 12 fields, got " + std::to_string(f.size()));
    }
    std::optional<models::State> state = sim::stateFromName(f[1]);
    if (!state) {
      throw std::runtime_error("run log line " + std::to_string(lineNumber) + ": unknown state '" + f[1] + "'");
    }
    // Steps before the simulated target exists have no ground truth; checkRun skips NaN.
    const double nan = std::numeric_limits<double>::quiet_NaN();
    sim::StepRecord step{
      .tS = toDouble(f[0], lineNumber, "t"),
      .state = *state,
      .targetValid = f[3] == "1",
      .trueBearingDeg = f[10].empty() ? nan : toDouble(f[10], lineNumber, "true_bearing_deg"),
      .trueDistanceM = f[11].empty() ? nan : toDouble(f[11], lineNumber, "true_distance_m"),
    };
    if (!f[8].empty() || !f[9].empty()) {
      step.setpoint = models::VelocityCmd{.vx = toDouble(f[8], lineNumber, "vx"), .yawRate = toDouble(f[9], lineNumber, "yaw_rate")};
    }
    steps.push_back(step);
  }
  return steps;
}

}  // namespace follow::util
