#include "Types.h"
#include "interfaces/IDroneState.h"
#include "states/StateStopped.h"
#include "states/StateTurning.h"
#include "util.h"

#include <cmath>
#include <memory>

std::unique_ptr<IDroneState> StateTurning::execute(MissionContext &ctx)
{
  ctx.commandMode = TURNING;

  // Дивимось, чи потрібно завершити поворот і перейти до стану STOPPED.
  // Якщо кут між поточним курсом і бажаним курсом менший за поріг, переходимо до стану STOPPED.
  // Інакше залишаємося в стані TURNING.
  float angleDelta = util::normalizeAngle(ctx.targetAngle - ctx.droneAngle);
  ctx.timeToStop = fabsf(angleDelta) / ctx.cfg.angularSpeed;

  if (fabsf(angleDelta) < 0.01f) {
    return std::make_unique<StateStopped>();
  }
  return std::make_unique<StateTurning>();
}