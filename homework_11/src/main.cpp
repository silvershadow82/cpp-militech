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

  configLoader->load();
  DroneConfig config = configLoader->getConfig();

  auto solver = componentFactory.createSolver(SolverType::TABLE);
  auto providerUniquePtr =
    componentFactory.createProvider(ProviderType::JSON, dataFolder + "/targets.json", config.arrayTimeStep, config.timeScale);
  auto physics = componentFactory.createDronePhysics(config);

  // Щоб не втратити доступ до об'єктів після передачі у MissionProcessor, зберігаємо сирі вказівники на них
  // динамічно приведений до потрібного типу, щоб не плодити зайві методи в інтерфейсі
  auto *provider = dynamic_cast<ThreadSafeTargetProvider *>(providerUniquePtr.get());
  auto *physicsHandle = physics.get();

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

  // чекаємо, поки обидва потоки не будуть готові до роботи
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
