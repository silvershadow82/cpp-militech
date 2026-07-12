#pragma once

#include "DronePhysics.h"
#include "Types.h"
#include "comms/SerialLink.h"
#include "control/FlightController.h"
#include "gpio/IGpioController.h"
#include "models/FireGeometry.h"
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
  std::unique_ptr<ITargetProvider> createProvider(ProviderType providerType, int nTargets, float timeScale = 1.0f);
  std::unique_ptr<IConfigLoader> createLoader(LoaderType loaderType, const std::string &param);
  std::unique_ptr<DronePhysics> createDronePhysics(const DroneConfig &config);
  std::shared_ptr<comms::SerialLink> createSerialLink();
  std::shared_ptr<gpio::IGpioController> createGpioController();
  std::unique_ptr<FlightController> createFlightController(const DroneConfig &config);
  std::unique_ptr<FireGeometry> createFireGeometry(const DroneConfig &config, std::unique_ptr<IBallisticSolver> solver);
};
