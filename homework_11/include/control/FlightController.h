#pragma once

#include "Types.h"
#include "models/Telem.h"
#include "models/drone_link.h"

class FlightController {
private:
  DroneConfig cfg;
  float velocityGain;

public:
  FlightController(const DroneConfig &cfg);
  dlink::Control compute(const Telem &telem, float desiredAngle, float desiredSpeed) const;
};
