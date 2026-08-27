#include "comms/MavLink.h"
#include "debug.h"
#include "Types.h"
#include "UartMissionProcessor.h"
#include "config/ComponentFactory.h"
#include "config/UartConfigLoader.h"
#include "models/FireGeometry.h"
#include "providers/UartTargetProvider.h"

#include <atomic>
#include <cstdlib>
#include <string>
#include <chrono>
#include <csignal>
#include <exception>
#include <iostream>
#include <thread>
#include <utility>

std::atomic<bool> stopRequested{false};

void handleSigint(int)
{
  stopRequested.store(true);
}

constexpr const char *DEFAULT_UART_DEVICE = "/dev/ttyAMA3";
constexpr const char *DEFAULT_GPIO_CHIP = "gpiochip0";
constexpr int START_LINE = 24;
constexpr int DROP_LINE = 23;
constexpr int DEFAULT_MAVLINK_PORT = 14055;
// ТЗ: адреса призначення за замовчуванням - те, що слухає QGroundControl.
constexpr const char *DEFAULT_MAVLINK_REMOTE_HOST = "127.0.0.1";
constexpr int DEFAULT_MAVLINK_REMOTE_PORT = 14550;

constexpr std::chrono::milliseconds configInitTimeout{5000};
constexpr std::chrono::milliseconds sleepTime{5};

constexpr std::chrono::milliseconds mavlinkPeriod{1000};
constexpr std::chrono::milliseconds mavlinkSleepSlice{50};

void interruptibleSleep(std::chrono::milliseconds total)
{
  auto deadline = std::chrono::steady_clock::now() + total;
  while (!stopRequested.load() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(mavlinkSleepSlice);
  }
}

int main(int argc, char **argv)
{
  // Без цього handleSigint ніколи не викликався і stopRequested лишався false.
  std::signal(SIGINT, handleSigint);
  std::signal(SIGTERM, handleSigint);

  const char *uartDevice = DEFAULT_UART_DEVICE;
  const char *gpioChip = DEFAULT_GPIO_CHIP;
  int mavlinkPort = DEFAULT_MAVLINK_PORT;
  std::string mavlinkRemoteHost = DEFAULT_MAVLINK_REMOTE_HOST;
  int mavlinkRemotePort = DEFAULT_MAVLINK_REMOTE_PORT;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const bool hasValue = (i + 1) < argc;
    if (arg == "--uart" && hasValue) {
      uartDevice = argv[++i];
    }
    else if (arg == "--gpiochip" && hasValue) {
      gpioChip = argv[++i];
    }
    else if (arg == "--mavlink-port" && hasValue) {
      mavlinkPort = std::atoi(argv[++i]);
    }
    else if (arg == "--mavlink-remote" && hasValue) {
      const std::string endpoint = argv[++i];
      const auto colon = endpoint.rfind(':');
      if (colon == std::string::npos) {
        std::cerr << "--mavlink-remote expects HOST:PORT" << std::endl;
        return 2;
      }
      mavlinkRemoteHost = endpoint.substr(0, colon);
      mavlinkRemotePort = std::atoi(endpoint.c_str() + colon + 1);
    }
    else {
      std::cerr << "usage: " << argv[0] << " [--uart DEV] [--gpiochip CHIP] [--mavlink-port PORT]" << " [--mavlink-remote HOST:PORT]"
                << std::endl;
      return 2;
    }
  }

  ComponentFactory componentFactory;

  auto serial = componentFactory.createSerialLink();

  if (!serial->open(uartDevice)) {
    std::cerr << "Failed to open UART device: " << uartDevice << std::endl;
    return 1;
  }

  auto gpio = componentFactory.createGpioController();

  if (!gpio || !gpio->init(gpioChip, START_LINE, DROP_LINE)) {
    std::cerr << "Failed to init GPIO chip=" << gpioChip << " (build with -DUSE_GPIOD=ON on the Pi)" << std::endl;
    return 1;
  }

  gpio->setStart(true);  // поїхали

  auto configLoader = componentFactory.createLoader(LoaderType::UART, uartDevice);
  auto *rawConfigLoader = dynamic_cast<UartConfigLoader *>(configLoader.get());

  if (!rawConfigLoader) {
    std::cerr << "Failed to construct UartConfigLoader" << std::endl;
    return 1;
  }

  bool configReady = UartMissionProcessor::initConfig(*serial.get(), *rawConfigLoader, configInitTimeout);

  if (!configReady) {
    LOG("config init incomplete after " << configInitTimeout.count() << "ms");
  }

  if (!rawConfigLoader->hasConfig()) {
    std::cerr << "PKT_CONFIG never arrived -- aborting" << std::endl;
    return 1;
  }

  DroneConfig config = rawConfigLoader->getConfig();

  auto targetProvider = componentFactory.createProvider(ProviderType::UART, static_cast<int>(rawConfigLoader->targetCount()));

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

  auto mavLink = componentFactory.createMavLink(mavlinkPort, mavlinkRemoteHost, mavlinkRemotePort);
  if (!mavLink->isOpen()) {
    std::cerr << "Failed to open MAVLink UDP socket on port " << mavlinkPort << std::endl;
    return 1;
  }
  if (!mavlinkRemoteHost.empty() && !mavLink->hasPeer()) {
    // Інакше програма мовчки працює далі й не шле нікуди жодного пакета.
    std::cerr << "Failed to resolve --mavlink-remote " << mavlinkRemoteHost << ":" << mavlinkRemotePort << std::endl;
    return 1;
  }
  LOG("MAVLink UDP listening on port " << mavlinkPort
                                       << (mavlinkRemoteHost.empty()
                                             ? " (peer learned from rx)"
                                             : " -> " + mavlinkRemoteHost + ":" + std::to_string(mavlinkRemotePort)));

  std::thread mavLinkPollThread(
    [](comms::MavLink *m) {
      while (!stopRequested.load()) {
        int parsed = m->rx_poll();
        if (parsed > 0) {
          DEBUG("MAVLink rx_poll: " << parsed << " message(s)");
        }
        interruptibleSleep(mavlinkPeriod);
      }
    },
    mavLink.get());

  std::thread mavLinkHeartbeatThread(
    [](comms::MavLink *m) {
      while (!stopRequested.load()) {
        m->send_heartbeat();
        interruptibleSleep(mavlinkPeriod);
      }
    },
    mavLink.get());

  // Створюємо процесор місій тут
  UartMissionProcessor missionProcessor(
    serial, gpio, std::move(uartConfigLoader), std::move(uartTargetProvider), std::move(geometry), std::move(flightController), mavLink);

  LOG("UartMissionProcessor started: uart=" << uartDevice << " gpiochip=" << gpioChip << " startLine=" << START_LINE
                                            << " dropLine=" << DROP_LINE);

  while (!stopRequested.load() && !missionProcessor.isComplete()) {
    missionProcessor.step(std::chrono::steady_clock::now());
    std::this_thread::sleep_for(sleepTime);
  }

  gpio->setStart(false);
  gpio->setDrop(false);

  LOG("Shutting down");

  stopRequested.store(true);

  mavLinkPollThread.join();
  mavLinkHeartbeatThread.join();

  return 0;
}
