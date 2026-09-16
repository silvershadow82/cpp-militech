#pragma once

#include "follow/core/Core.h"
#include "follow/core/Types.h"
#include "follow/runtime/EventQueue.h"
#include "follow/runtime/Latest.h"

namespace follow::runtime {

// Ground truth published by the simulated camera for the run log.
struct SimTruth {
  double bearingDeg{0.0};
  double distanceM{0.0};
};

// Everything the threads of follow_app share. Each field has one writer:
//   vehicle, (setpoint is read)  - MAVLink I/O thread
//   observation, truth           - vision thread
//   setpoint, trackerRequests    - control loop (trackerRequests is drained by the vision thread)
struct Channels {
  Latest<core::VehicleState> vehicle;
  Latest<core::TargetObservation> observation;
  Latest<SimTruth> truth;
  Latest<core::VelocityCmd> setpoint;                // written only when the core commands a setpoint
  EventQueue<core::TrackerRequest> trackerRequests;  // edge events: must not be overwritten
};

}  // namespace follow::runtime
