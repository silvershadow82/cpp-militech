#pragma once

#include "DronePhysics.h"
#include "Types.h"
#include <memory>
#include <string>

class IBallisticSolver;
class ITargetProvider;
class IConfigLoader;

class ComponentFactory {
public:
  std::unique_ptr<IBallisticSolver> createSolver(SolverType solverType);
  std::unique_ptr<ITargetProvider> createProvider(ProviderType providerType,
                                                  const std::string &param,
                                                  float arrayTimeStep,
                                                  float timeScale);
  std::unique_ptr<IConfigLoader> createLoader(LoaderType loaderType, const std::string &param);
  std::unique_ptr<DronePhysics> createDronePhysics(const DroneConfig &config);
};
