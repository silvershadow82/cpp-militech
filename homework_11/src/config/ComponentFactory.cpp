
#include "Types.h"
#include "config/ComponentFactory.h"
#include "config/FileConfigLoader.h"
#include "config/UartConfigLoader.h"
#include "control/FlightController.h"
#include "gpio/IGpioController.h"
#include "interfaces/IBallisticSolver.h"
#include "models/FireGeometry.h"
#include "providers/ThreadSafeTargetProvider.h"
#include "providers/UartTargetProvider.h"
#include "solvers/AnalyticalSolver.h"
#include "solvers/TableSolver.h"
#include "DronePhysics.h"
#include <cstddef>
#include <memory>
#include <utility>

std::unique_ptr<IBallisticSolver> ComponentFactory::createSolver(SolverType solverType)
{
  std::unique_ptr<IBallisticSolver> solver = nullptr;
  switch (solverType) {
    case SolverType::ANALYTICAL:
      solver = std::make_unique<AnalyticalSolver>();
      break;
    case SolverType::TABLE:
      solver = std::make_unique<TableSolver>();
      break;
    default:
      break;
  }
  return solver;
}

std::unique_ptr<ITargetProvider> ComponentFactory::createProvider(ProviderType providerType,
                                                                  const std::string &param,
                                                                  float arrayTimeStep,
                                                                  float timeScale)
{
  std::unique_ptr<ITargetProvider> provider = nullptr;
  switch (providerType) {
    case ProviderType::JSON:
      provider = std::make_unique<ThreadSafeTargetProvider>(param, arrayTimeStep, timeScale);
      break;
    default:
      break;
  }
  return provider;
}

std::unique_ptr<ITargetProvider> ComponentFactory::createProvider(ProviderType providerType, int nTargets, float timeScale)
{
  std::unique_ptr<ITargetProvider> provider = nullptr;
  switch (providerType) {
    case ProviderType::UART:
      provider = std::make_unique<UartTargetProvider>(nTargets, timeScale);
      break;
    default:
      break;
  }
  return provider;
}

std::unique_ptr<IConfigLoader> ComponentFactory::createLoader(LoaderType loaderType, const std::string &param)
{
  std::unique_ptr<IConfigLoader> loader = nullptr;
  switch (loaderType) {
    case LoaderType::FILE:
      loader = std::make_unique<FileConfigLoader>(param + "/config.json", param + "/ammo.json");
      break;
    case LoaderType::UART:
      loader = std::make_unique<UartConfigLoader>();
      break;
    default:
      break;
  }
  return loader;
}

std::unique_ptr<DronePhysics> ComponentFactory::createDronePhysics(const DroneConfig &config)
{
  return std::make_unique<DronePhysics>(config);
}

std::unique_ptr<gpio::IGpioController> ComponentFactory::createGpioController()
{
#if defined(USE_GPIOD)
  return std::make_unique<gpio::LibGpiodController>();
#else
  return nullptr;
#endif
}

std::unique_ptr<FlightController> ComponentFactory::createFlightController(const DroneConfig &config)
{
  return std::make_unique<FlightController>(config);
}

std::unique_ptr<FireGeometry> ComponentFactory::createFireGeometry(const DroneConfig &config, std::unique_ptr<IBallisticSolver> solver)
{
  return std::make_unique<FireGeometry>(config, std::move(solver));
}