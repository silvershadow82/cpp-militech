#pragma once

#include <string>
#include "IBallisticSolver.hpp"
#include "IConfigLoader.hpp"
#include "ITargetProvider.hpp"

enum class SolverType { ANALYTICAL };
enum class ProviderType { JSON };
enum class LoaderType { FILE };
enum class ConfigSource { FILE };

class ComponentFactory {
public:
  IBallisticSolver* createSolver(SolverType solverType);
  ITargetProvider* createProvider(ProviderType providerType, const std::string& param);
  IConfigLoader* createLoader(LoaderType loaderType, const std::string& param);
};
