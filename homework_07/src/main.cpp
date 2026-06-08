#include "debug.hpp"
#include "MissionProcessor.hpp"
#include <iostream>
#include <cstring>

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
