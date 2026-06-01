#include "common.hpp"
#include "debug.hpp"
#include "factory.hpp"
#include "mission_processor.hpp"
#include <iostream>
#include <cstring>
#include <cmath>

char *getDroneStateName(DroneState droneState)
{
  static char name[32];

  switch (droneState) {
    case DroneState::STOPPED:
      std::strcpy(name, "[STOPPED]");
      break;
    case DroneState::ACCELERATING:
      std::strcpy(name, "[ACCELERATING]");
      break;
    case DroneState::DECELERATING:
      std::strcpy(name, "[DECELERATING]");
      break;
    case DroneState::MOVING:
      std::strcpy(name, "[MOVING]");
      break;
    case DroneState::TURNING:
      std::strcpy(name, "[TURNING]");
      break;
    default:
      break;
  }
  return name;
}

int main(int argc, char **argv)
{
  if (argc != 2) {
    std::cout << "Usage simulation <data_folder>" << '\n';
    return 0;
  }

  const std::string dataFolder = argv[1];

  MissionProcessor missionProcessor = MissionProcessor();

  if (!missionProcessor.init(ConfigSource::FILE, dataFolder)) {
    return 1;
  }

  while (missionProcessor.hasNext()) {
    if (!missionProcessor.step()) {
      break;
    }
  }

  missionProcessor.printStats();

  LOG("Simulation complete. Steps=" << missionProcessor.totalSteps());

  return 0;
}
