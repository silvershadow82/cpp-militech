#pragma once

#include "Types.h"
#include "comms/Frame.h"
#include "interfaces/IConfigLoader.h"
#include "models/Telem.h"
#include "models/drone_link.h"
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>

class UartConfigLoader : public IConfigLoader
{
private:
  bool ammoReceived{false};
  bool configReceived{false};
  std::optional<Telem> firstTelemetry{};
  dlink::AmmoCfg ammoCfg{};
  dlink::DroneCfg droneCfg{};

  DroneConfig droneConfig{};
  std::unordered_map<std::string, AmmoParams> ammoParams{};

  void rebuild();

public:
  UartConfigLoader() = default;
  ~UartConfigLoader() override = default;
  // Зчитує кадр протоколу drone_link і накопичує стан.
  // Підтримує тільки PKT_AMMO, PKT_CONFIG, PKT_TELEMETRY
  void feed(const comms::Frame &frame);

  bool isReady() const;
  bool hasConfig() const;

  // Встановлює перший отриманий PKT_TELEMETRY
  // для ініціалізації дрона (startPos, altitude, initialDir)
  std::optional<Telem> firstTelem() const;
  uint8_t targetCount() const;
  void load() override;
  DroneConfig getConfig() override;
  std::unordered_map<std::string, AmmoParams> getAmmoParams() override;
  int getAmmoCount() override;
};
