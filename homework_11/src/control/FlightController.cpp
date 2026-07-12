#include "control/FlightController.h"
#include "util.h"

#include <algorithm>
#include <cmath>

// Параметри контролера, що не залежать від конфігурації дрона
constexpr float maxAngleTurn = static_cast<float>(M_PI) / 2.0f;  // 90 deg
constexpr float angleTurnGain = 1.0f / maxAngleTurn;
// Мінімальна швидкість атаки, нижче якої не можна ділити (щоб уникнути ділення на нуль).
constexpr float minAttackSpeed = 1.0f;

FlightController::FlightController(const DroneConfig &config)
  : cfg(config)
  , velocityGain(1.0f / std::max(config.attackSpeed, minAttackSpeed))
{
}

dlink::Control FlightController::compute(const Telem &telem, float desiredAngle, float desiredSpeed) const
{
  float angleError = util::normalizeAngle(desiredAngle - telem.angle);
  float speedError = desiredSpeed - telem.speed;

  dlink::Control control{};
  control.turnRate = std::clamp(angleTurnGain * angleError, -1.0f, 1.0f);
  control.accel = std::clamp(this->velocityGain * speedError, -1.0f, 1.0f);
  return control;
}
