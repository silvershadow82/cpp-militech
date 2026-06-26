#include "ComponentFactory.h"
#include "config/FileConfigLoader.h"
#include "providers/JsonTargetProvider.h"
#include "solvers/AnalyticalSolver.h"

IBallisticSolver *ComponentFactory::createSolver(SolverType solverType) {
  IBallisticSolver *solver = nullptr;
  switch (solverType) {
  case SolverType::ANALYTICAL:
    solver = new AnalyticalSolver();
    break;
  default:
    break;
  }
  return solver;
}

ITargetProvider *ComponentFactory::createProvider(ProviderType providerType,
                                                  const std::string &param) {
  ITargetProvider *provider = nullptr;
  switch (providerType) {
  case ProviderType::JSON:
    provider = new JsonTargetProvider(param);
    break;
  default:
    break;
  }
  return provider;
}

IConfigLoader *ComponentFactory::createLoader(LoaderType loaderType,
                                              const std::string &param) {
  IConfigLoader *loader = nullptr;
  switch (loaderType) {
  case LoaderType::FILE:
    loader = new FileConfigLoader(param + "/config.json", param + "/ammo.json");
    break;
  default:
    break;
  }
  return loader;
}