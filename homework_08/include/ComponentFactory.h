#pragma once

#include "Types.h"
#include <string>

class IBallisticSolver;
class ITargetProvider;
class IConfigLoader;

class ComponentFactory {
public:
  IBallisticSolver *createSolver(SolverType solverType);
  ITargetProvider *createProvider(ProviderType providerType,
                                  const std::string &param);
  IConfigLoader *createLoader(LoaderType loaderType, const std::string &param);
};
