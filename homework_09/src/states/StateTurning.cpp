#include "Types.h"
#include "interfaces/IDroneState.h"
#include "states/StateStopped.h"
#include "states/StateTurning.h"
#include "util.h"

#include <cmath>
#include <memory>

std::unique_ptr<IDroneState> StateTurning::execute(DroneContext &ctx)
{
  ctx.state = this->name();

  float angleDelta = util::normalizeAngle(ctx.targetAngle - ctx.droneAngle);
  ctx.timeToStop = fabsf(angleDelta) / ctx.cfg.angularSpeed;

  if (util::convergeAngle(ctx.droneAngle, ctx.targetAngle, ctx.cfg) < 0.01) {
    return std::make_unique<StateStopped>();
  }
  return std::make_unique<StateTurning>();
}