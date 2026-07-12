#include "UartMissionProcessor.h"
#include "config/UartConfigLoader.h"
#include "debug.h"
#include "interfaces/IConfigLoader.h"
#include "models/Coord.h"
#include "models/Target.h"
#include <chrono>
#include <limits>
#include <memory>
#include <thread>

using TimeUnit = std::chrono::milliseconds;

UartMissionProcessor::UartMissionProcessor(std::shared_ptr<comms::SerialLink> serial,
                                           std::shared_ptr<gpio::IGpioController> gpio,
                                           std::unique_ptr<UartConfigLoader> configLoader,
                                           std::unique_ptr<UartTargetProvider> targetProvider,
                                           std::unique_ptr<FireGeometry> fireGeometry,
                                           std::unique_ptr<FlightController> flightController,
                                           UartMissionProcessorParams params)
  : serial(serial)
  , gpio(gpio)
  , configLoader(std::move(configLoader))
  , targetProvider(std::move(targetProvider))
  , fireGeometry(std::move(fireGeometry))
  , flightController(std::move(flightController))
  , params(params)
{
}

bool UartMissionProcessor::initConfig(comms::SerialLink &serial, UartConfigLoader &loader, TimeUnit timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!loader.isReady() && std::chrono::steady_clock::now() < deadline) {
    bool frameReady = false;
    auto frame = serial.readFrame();
    while (frame.ok) {
      loader.feed(frame);
      frameReady = true;
      frame = serial.readFrame();
    }
    if (!frameReady) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  return loader.isReady();
}

void UartMissionProcessor::processFrame(const comms::Frame &frame, Clock::time_point now)
{
  switch (frame.type) {
    case dlink::PKT_TELEMETRY: {
      dlink::Telemetry t = frame.as<dlink::Telemetry>();
      this->telem.pos = Coord{t.x, t.y};
      this->telem.altitude = t.z;
      this->telem.speed = t.speed;
      this->telem.angle = t.dir;
      this->telem.t_ms = t.t_ms;
      this->telemetrySeen = true;
      this->lastRxTime = now;
      break;
    }
    case dlink::PKT_TARGET:
      if (this->targetProvider) {
        this->targetProvider->update(frame.as<dlink::TargetPos>());
      }
      break;
    case dlink::PKT_RESULT: {
      dlink::Result result = frame.as<dlink::Result>();
      LOG("PKT_RESULT: " << (result.hit ? "HIT" : "MISS") << " targetId=" << result.targetId << " miss_m=" << result.miss_m
                         << " drop_t_ms=" << result.drop_t_ms);
      break;
    }
    default:
      break;
  }
}

void UartMissionProcessor::updateGuidance(Clock::time_point now)
{
  const bool haveUsableTargets =
    this->targetProvider && this->fireGeometry && this->targetProvider->allSeen() && this->targetProvider->getTargetCount() > 0;

  if (!haveUsableTargets) {
    this->lastControl = this->flightController->compute(this->telem, this->telem.angle, 0.0f);
    return;
  }

  int bestIndex = -1;
  FireSolution best{};
  float bestDist = std::numeric_limits<float>::infinity();

  for (int i = 0; i < this->targetProvider->getTargetCount(); ++i) {
    Target target = this->targetProvider->getTarget(i);
    FireSolution solution = this->fireGeometry->solve(this->telem, target);
    if (!solution.ok) {
      continue;
    }
    float dist = Coord::distance(this->telem.pos, solution.dropPoint);
    if (dist < bestDist) {
      bestDist = dist;
      best = solution;
      bestIndex = i;
    }
  }

  if (bestIndex < 0) {
    this->lastControl = this->flightController->compute(this->telem, this->telem.angle, 0.0f);
    return;
  }

  float desiredSpeed = this->dropped ? 0.0f : best.desiredSpeed;
  this->lastControl = this->flightController->compute(this->telem, best.aimAngle, desiredSpeed);

  if (best.inDropWindow && !this->dropped) {
    this->gpio->setDrop(true);
    this->dropActive = true;
    this->dropOffAt = now + this->params.dropPulseDuration;
    this->dropped = true;
  }
}

void UartMissionProcessor::step(Clock::time_point now)
{
  auto frame = this->serial->readFrame();

  while (frame.ok) {
    this->processFrame(frame, now);
    frame = this->serial->readFrame();
  }

  if (this->dropActive && now >= this->dropOffAt) {
    this->gpio->setDrop(false);
    this->dropActive = false;
  }

  if (this->telemetrySeen) {
    if (now - this->lastRxTime > this->params.telemetryWatchdog) {
      this->lastControl = dlink::Control{0.0f, 0.0f};  // telemetry lost -> neutral control
    }
    else {
      this->updateGuidance(now);
    }
  }

  if (!this->txPrimed || (now - this->lastTxTime) >= this->params.controlPeriod) {
    this->serial->sendControl(this->lastControl.accel, this->lastControl.turnRate);
    this->lastTxTime = now;
    this->txPrimed = true;
  }
}
