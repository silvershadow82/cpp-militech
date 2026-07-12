#include "config/UartConfigLoader.h"
#include "models/drone_link.h"

void UartConfigLoader::feed(const comms::Frame &frame)
{
  switch (frame.type) {
    case dlink::PKT_AMMO:
      this->ammoCfg = frame.as<dlink::AmmoCfg>();
      this->ammoReceived = true;
      this->rebuild();
      break;
    case dlink::PKT_CONFIG:
      this->droneCfg = frame.as<dlink::DroneCfg>();
      this->configReceived = true;
      this->rebuild();
      break;
    case dlink::PKT_TELEMETRY:
      if (!this->firstTelemetry.has_value()) {
        dlink::Telemetry telemetry = frame.as<dlink::Telemetry>();
        Telem telem{};
        telem.pos = Coord{telemetry.x, telemetry.y};
        telem.altitude = telemetry.z;
        telem.speed = telemetry.speed;
        telem.angle = telemetry.dir;
        telem.t_ms = telemetry.t_ms;
        this->firstTelemetry = telem;
        this->rebuild();
      }
      break;
    default:
      break;
  }
}

bool UartConfigLoader::isReady() const
{
  return this->ammoReceived && this->configReceived && this->firstTelemetry.has_value();
}

bool UartConfigLoader::hasConfig() const
{
  return this->configReceived;
}

std::optional<Telem> UartConfigLoader::firstTelem() const
{
  return this->firstTelemetry;
}

uint8_t UartConfigLoader::targetCount() const
{
  return this->ammoReceived ? this->ammoCfg.nTargets : 0;
}

void UartConfigLoader::rebuild()
{
  DroneConfig config{};

  if (this->firstTelemetry.has_value()) {
    config.startPos = this->firstTelemetry->pos;
    config.altitude = this->firstTelemetry->altitude;
    config.initialDir = this->firstTelemetry->angle;
  }

  if (this->ammoReceived) {
    config.ammoName = this->ammoCfg.name;
    config.hitRadius = this->ammoCfg.hitRadius;

    this->ammoParams.clear();
    this->ammoParams.emplace(
      this->ammoCfg.name,
      AmmoParams{.name = this->ammoCfg.name, .mass = this->ammoCfg.mass, .drag = this->ammoCfg.drag, .lift = this->ammoCfg.lift});
  }

  if (this->configReceived) {
    config.attackSpeed = this->droneCfg.attackSpeed;
    config.accelPath = this->droneCfg.accelerationPath;
    config.angularSpeed = this->droneCfg.angularSpeed;
    config.turnThreshold = this->droneCfg.turnThreshold;
    config.simTimeStep = this->droneCfg.timeStep;
    config.timeScale = this->droneCfg.timeScale;
  }

  this->droneConfig = config;
}

void UartConfigLoader::load()
{
  // не використовується, бо UartConfigLoader накопичує стан через feed() і rebuild() у реальному часі
}

DroneConfig UartConfigLoader::getConfig()
{
  return this->droneConfig;
}

std::unordered_map<std::string, AmmoParams> UartConfigLoader::getAmmoParams()
{
  return this->ammoParams;
}

int UartConfigLoader::getAmmoCount()
{
  return static_cast<int>(this->ammoParams.size());
}
