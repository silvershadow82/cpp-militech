#include "ComponentFactory.h"
#include "Types.h"
#include "config/FileConfigLoader.h"
#include "providers/ThreadSafeTargetProvider.h"
#include "solvers/AnalyticalSolver.h"
#include "solvers/TableSolver.h"
#include <memory>

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

std::unique_ptr<ITargetProvider> ComponentFactory::createProvider(ProviderType providerType, const std::string &param)
{
  std::unique_ptr<ITargetProvider> provider = nullptr;
  switch (providerType) {
    case ProviderType::JSON:
      // TODO: populate real arrayTimeStep from config or parameter
      provider = std::make_unique<ThreadSafeTargetProvider>(param, 5);
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
    default:
      break;
  }
  return loader;
}