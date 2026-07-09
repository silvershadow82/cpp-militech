#include "Types.h"
#include "interfaces/IDroneState.h"
#include "states/StateAccelerating.h"
#include "states/StateTurning.h"
#include "states/StateStopped.h"
#include "util.h"

#include <cmath>
#include <memory>

std::unique_ptr<IDroneState> StateStopped::execute(MissionContext &ctx)
{
  ctx.commandMode = STOPPED;

  // Дивимось, чи потрібно розпочати прискорення або поворот.
  // Якщо кут між поточним курсом і бажаним курсом перевищує поріг, переходимо до стану TURNING.
  // Інакше переходимо до стану ACCELERATING.
  float angleDelta = util::normalizeAngle(ctx.targetAngle - ctx.droneAngle);

  if (fabsf(angleDelta) > ctx.cfg.turnThreshold) {
    ctx.timeToStop = fabsf(angleDelta) / ctx.cfg.angularSpeed;
    return std::make_unique<StateTurning>();
  }
  ctx.timeToStop = 0.f;
  return std::make_unique<StateAccelerating>();
}