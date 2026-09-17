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

// Everything the threads of follow_app share. While the threads run, each field has one writer:
//   vehicle, (setpoint is read)       - MAVLink I/O thread
//   observation, truth                - vision thread
//   setpoint, trackerRequests, overlay - control loop (trackerRequests is drained by the vision thread)
struct Channels {
  Latest<core::VehicleState> vehicle;
  Latest<core::TargetObservation> observation;
  Latest<SimTruth> truth;
  // Written by the control loop when the core commands a setpoint. runHwApp's shutdown writes it
  // once more -- the zero fail-safe -- but only after the control thread has been joined, so that
  // write is still the last one the I/O thread sees. Keep that ordering if shutdown is ever changed.
  Latest<core::VelocityCmd> setpoint;
  EventQueue<core::TrackerRequest> trackerRequests;  // edge events: must not be overwritten
  Latest<core::OverlayInfo> overlay;                 // what the pilot's video overlay shows, every tick
};

}  // namespace follow::runtime
