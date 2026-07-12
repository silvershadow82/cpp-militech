#include "debug.h"
#include "Types.h"
#include "UartMissionProcessor.h"
#include "config/ComponentFactory.h"
#include "config/UartConfigLoader.h"
#include "interfaces/IBallisticSolver.h"
#include "models/FireGeometry.h"
#include "providers/UartTargetProvider.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <exception>
#include <iostream>
#include <memory>
#include <thread>
#include <utility>

std::atomic<bool> stopRequested{false};

void handleSigint(int)
{
  stopRequested.store(true);
}

constexpr const char *UART_DEVICE = "/dev/serial0";
constexpr const char *GPIO_CHIP = "gpiochip0";
constexpr int START_LINE = 24;
constexpr int DROP_LINE = 23;

constexpr std::chrono::milliseconds configInitTimeout{5000};
constexpr std::chrono::milliseconds sleepTime{5};

int main(int argc, char **argv)
{
  ComponentFactory componentFactory;

  auto serial = componentFactory.createSerialLink();

  if (!serial->open(UART_DEVICE)) {
    std::cerr << "Failed to open UART device: " << UART_DEVICE << std::endl;
    return 1;
  }

  auto gpio = componentFactory.createGpioController();

  if (!gpio || !gpio->init(GPIO_CHIP, START_LINE, DROP_LINE)) {
    std::cerr << "Failed to init GPIO chip=" << GPIO_CHIP << std::endl;
    return 1;
  }

  gpio->setStart(true);  // поїхали

  auto configLoader = componentFactory.createLoader(LoaderType::UART, UART_DEVICE);
  auto *rawConfigLoader = dynamic_cast<UartConfigLoader *>(configLoader.get());

  if (!rawConfigLoader) {
    std::cerr << "Failed to construct UartConfigLoader" << std::endl;
    return 1;
  }

  bool configReady = UartMissionProcessor::initConfig(*serial.get(), *rawConfigLoader, configInitTimeout);

  if (!rawConfigLoader->firstTelem().has_value()) {
    std::cerr << "No PKT_TELEMETRY received within " << configInitTimeout.count() << "ms -- aborting" << std::endl;
    return 1;
  }

  if (!configReady) {
    LOG("config init incomplete after " << configInitTimeout.count() << "ms");
  }

  if (!rawConfigLoader->hasConfig()) {
    std::cerr << "PKT_CONFIG never arrived -- aborting" << std::endl;
    return 1;
  }

  DroneConfig config = rawConfigLoader->getConfig();

  auto targetProvider =
    componentFactory.createProvider(ProviderType::UART, static_cast<int>(rawConfigLoader->targetCount()), config.timeScale);

  std::unique_ptr<FireGeometry> geometry;
  if (rawConfigLoader->hasConfig()) {
    auto ammoParams = rawConfigLoader->getAmmoParams();
    if (ammoParams.contains(config.ammoName)) {
      try {
        auto solver = componentFactory.createSolver(SolverType::TABLE);
        solver->init(config, ammoParams.at(config.ammoName).payloadParams());
        geometry = componentFactory.createFireGeometry(config, std::move(solver));
      }
      catch (const std::exception &e) {
        std::cerr << "TableSolver::init failed (" << e.what() << ") -- running FlightController fallback" << std::endl;
        geometry.reset();
      }
    }
    else {
      std::cerr << "Unknown ammo '" << config.ammoName << "' -- running FlightController fallback." << std::endl;
    }
  }

  auto flightController = componentFactory.createFlightController(config);

  // Нам вже не потрібен інтерфейсний вказівник на configLoader, але потрібен вказівник на UartConfigLoader
  configLoader.release();
  std::unique_ptr<UartConfigLoader> uartConfigLoader(rawConfigLoader);
  // теж саме і targetProvider
  auto rawTargetProvider = dynamic_cast<UartTargetProvider *>(targetProvider.get());
  targetProvider.release();
  std::unique_ptr<UartTargetProvider> uartTargetProvider(rawTargetProvider);

  // Створюємо процесор місій тут
  UartMissionProcessor missionProcessor(
    serial, gpio, std::move(uartConfigLoader), std::move(uartTargetProvider), std::move(geometry), std::move(flightController));

  LOG("UartMissionProcessor started: uart=" << UART_DEVICE << " gpiochip=" << GPIO_CHIP << " startLine=" << START_LINE
                                            << " dropLine=" << DROP_LINE);

  while (!stopRequested.load()) {
    missionProcessor.step(std::chrono::steady_clock::now());
    std::this_thread::sleep_for(sleepTime);
  }

  LOG("Shutting down");
  return 0;
}
