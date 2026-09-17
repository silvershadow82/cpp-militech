#pragma once

#include <variant>
#include <vector>

#include "follow/core/Types.h"

namespace follow::sim {

struct TargetHold {
  double durationS{0.0};
};

struct TargetLine {
  double durationS{0.0};
  double velNorth{0.0};
  double velEast{0.0};
};

// Walks around centerNed at the given speed; positive speed is clockwise seen from above.
struct TargetCircle {
  double durationS{0.0};
  core::Vec3 centerNed{};
  double speed{0.0};
};

using TargetMotion = std::variant<TargetHold, TargetLine, TargetCircle>;

struct OcclusionWindow {
  double startS{0.0};
  double endS{0.0};
};

// Scripted target: a person-sized upright box moving on the ground (NED z = 0).
class SimTarget {
public:
  SimTarget(core::Vec3 startNed,
            std::vector<TargetMotion> motions,
            std::vector<OcclusionWindow> occlusions = {},
            double heightM = 1.7,
            double widthM = 0.5);

  // Ground point under the target at tSec after the scenario's target clock started.
  core::Vec3 positionAt(double tSec) const;
  bool occludedAt(double tSec) const;
  double height() const { return this->heightM; }
  double width() const { return this->widthM; }

private:
  core::Vec3 start;
  std::vector<TargetMotion> motions;
  std::vector<OcclusionWindow> occlusions;
  double heightM;
  double widthM;
};

}  // namespace follow::sim
