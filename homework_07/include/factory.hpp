#pragma once

#include <cstring>
#include "provider.hpp"

enum class SolverType { ANALYTICAL };
enum class ProviderType { JSON };
enum class LoaderType { FILE };
enum class ConfigSource { FILE };

class BallisticSolverFactory {
private:
  BallisticSolverFactory();

public:
  static IBallisticSolver* createSolver(SolverType solverType)
  {
    IBallisticSolver* solver = nullptr;
    switch (solverType) {
      case SolverType::ANALYTICAL:
        solver = new AnalyticalSolver();
        break;
      default:
        break;
    }
    return solver;
  }

  ~BallisticSolverFactory() = default;
};

class TargetProviderFactory {
private:
  TargetProviderFactory();

public:
  static ITargetProvider* createProvider(ProviderType providerType, const std::string& param)
  {
    ITargetProvider* provider = nullptr;
    switch (providerType) {
      case ProviderType::JSON:
        provider = new JsonTargetProvider(param);
        break;
      default:
        break;
    }
    return provider;
  }
};

class ConfigLoaderFactory {
private:
  ConfigLoaderFactory();

public:
  static IConfigLoader* createLoader(LoaderType loaderType, const std::string& param)
  {
    IConfigLoader* loader = nullptr;
    switch (loaderType) {
      case LoaderType::FILE:
        loader = new FileConfigLoader(param + "/config.json", param + "/ammo.json");
        break;
      default:
        break;
    }
    return loader;
  }
};
