#pragma once

#include "comms/Frame.h"
#include "comms/SerialLink.h"
#include "config/UartConfigLoader.h"
#include "control/FlightController.h"
#include "gpio/IGpioController.h"
#include "interfaces/ITargetProvider.h"
#include "models/FireGeometry.h"
#include "models/Telem.h"
#include "models/drone_link.h"
#include "providers/UartTargetProvider.h"
#include <chrono>
#include <memory>

using TimeUnit = std::chrono::milliseconds;

struct UartMissionProcessorParams {
  TimeUnit controlPeriod{20};       // fixed CONTROL emission cadence
  TimeUnit dropPulseDuration{300};  // DROP pulse width (widened from 50-100ms; checker's poll rate may miss shorter pulses)
  TimeUnit telemetryWatchdog{500};  // wall-clock loss-of-telemetry fail-safe
};

class UartMissionProcessor {
public:
  using Clock = std::chrono::steady_clock;

  UartMissionProcessor(std::shared_ptr<comms::SerialLink> serial,
                       std::shared_ptr<gpio::IGpioController> gpio,
                       std::unique_ptr<UartConfigLoader> configLoader,
                       std::unique_ptr<UartTargetProvider> targetProvider,
                       std::unique_ptr<FireGeometry> fireGeometry,
                       std::unique_ptr<FlightController> flightController,
                       UartMissionProcessorParams params = {});

  void step(Clock::time_point now);
  static bool initConfig(comms::SerialLink &serial, UartConfigLoader &configLoader, TimeUnit timeout);

private:
  void processFrame(const comms::Frame &frame, Clock::time_point now);
  void updateGuidance(Clock::time_point now);

  std::shared_ptr<comms::SerialLink> serial;
  std::shared_ptr<gpio::IGpioController> gpio;
  std::unique_ptr<UartConfigLoader> configLoader;
  std::unique_ptr<UartTargetProvider> targetProvider;
  std::unique_ptr<FireGeometry> fireGeometry;
  std::unique_ptr<FlightController> flightController;

  UartMissionProcessorParams params;

  Telem telem{};
  bool telemetrySeen{false};
  Clock::time_point lastRxTime{};

  bool txPrimed{false};
  Clock::time_point lastTxTime{};
  dlink::Control lastControl{};

  bool dropped{false};
  bool dropActive{false};
  Clock::time_point dropOffAt{};
};
