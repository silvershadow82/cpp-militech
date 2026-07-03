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

  // Decide-only: HW9 transitioned to Stopped when the pre-converge angle diff
  // dropped below the threshold (convergeAngle returned that pre-converge diff).
  float angleDelta = util::normalizeAngle(ctx.targetAngle - ctx.droneAngle);
  ctx.timeToStop = fabsf(angleDelta) / ctx.cfg.angularSpeed;

  if (fabsf(angleDelta) < 0.01f) {
    return std::make_unique<StateStopped>();
  }
  return std::make_unique<StateTurning>();
}