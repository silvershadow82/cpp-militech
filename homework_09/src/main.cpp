#include "ComponentFactory.h"
#include "MissionProcessor.h"
#include "debug.h"
#include <cstring>
#include <iostream>

int main(int argc, char **argv)
{
  if (argc != 2) {
    std::cout << "Usage simulation <data_folder>" << '\n';
    return 0;
  }

  const std::string dataFolder = argv[1];

  ComponentFactory componentFactory = ComponentFactory();

  auto configLoader = componentFactory.createLoader(LoaderType::FILE, dataFolder);
  auto solver = componentFactory.createSolver(SolverType::ANALYTICAL);
  auto targetProvider = componentFactory.createProvider(ProviderType::JSON, dataFolder + "/targets.json");

  auto missionProcessor = MissionProcessor(std::move(solver), std::move(targetProvider), std::move(configLoader));

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
