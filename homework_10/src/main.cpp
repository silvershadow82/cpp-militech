#include "config/ComponentFactory.h"
#include "DronePhysics.h"
#include "MissionProcessor.h"
#include "StatCollector.h"
#include "debug.h"
#include "interfaces/IBallisticSolver.h"
#include "interfaces/IConfigLoader.h"
#include "interfaces/ITargetProvider.h"
#include "providers/ThreadSafeTargetProvider.h"
#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>

int main(int argc, char **argv)
{
  if (argc != 2) {
    std::cout << "Usage simulation <data_folder>" << '\n';
    return 0;
  }

  const std::string dataFolder = argv[1];

  ComponentFactory componentFactory = ComponentFactory();

  auto configLoader = componentFactory.createLoader(LoaderType::FILE, dataFolder);

  if (!configLoader) {
    LOG("Error: failed to create config loader");
    return 1;
  }
  // load config before constructing physics-dependent components.
  configLoader->load();
  DroneConfig config = configLoader->getConfig();

  auto solver = componentFactory.createSolver(SolverType::TABLE);
  auto providerUniquePtr =
    componentFactory.createProvider(ProviderType::JSON, dataFolder + "/targets.json", config.arrayTimeStep, config.timeScale);
  auto physics = componentFactory.createDronePhysics(config);

  // Capture raw handles BEFORE moving ownership into the MissionProcessor.
  // ITargetProvider is polymorphic (virtual dtor), so dynamic_cast is valid.
  auto *provider = dynamic_cast<ThreadSafeTargetProvider *>(providerUniquePtr.get());
  DronePhysics *physicsHandle = physics.get();  // raw handle captured BEFORE the move below (moved-from unique_ptr is empty)

  // Validate handles BEFORE moving ownership into MissionProcessor (afterwards
  // the unique_ptrs are empty and these raw pointers would be moot).
  if (provider == nullptr || !physics) {
    LOG("Error: failed to construct provider/physics components (prov=" << provider << ", phys=" << physics.get() << ")");
    return 1;
  }

  MissionProcessor missionProcessor(std::move(solver), std::move(providerUniquePtr), std::move(configLoader), std::move(physics));

  if (!missionProcessor.init(ConfigSource::FILE, dataFolder)) {
    return 1;
  }

  std::thread providerThread(&ThreadSafeTargetProvider::run, provider);
  std::thread physicsThread(&DronePhysics::run, physicsHandle);

  // Wait until both worker threads report ready
  while (!(provider->isThreadReady() && physicsHandle->isThreadReady())) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  provider->start();
  physicsHandle->start();

  std::thread missionThread(&MissionProcessor::run, &missionProcessor);

  missionThread.join();
  LOG("Mission thread joined.");

  provider->stop();
  physicsHandle->stop();

  providerThread.join();
  LOG("Provider thread joined.");
  physicsThread.join();
  LOG("Physics thread joined.");

  missionProcessor.printStats();

  LOG("Simulation complete. Steps=" << missionProcessor.totalSteps());

  return 0;
}
