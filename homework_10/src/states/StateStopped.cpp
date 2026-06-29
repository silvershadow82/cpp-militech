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
  ctx.state = this->name();
  // Check to see if we need to turn towards the selected target
  float angleDelta = util::normalizeAngle(ctx.targetAngle - ctx.droneAngle);

  if (fabsf(angleDelta) > ctx.cfg.turnThreshold) {
    ctx.timeToStop = fabsf(angleDelta) / ctx.cfg.angularSpeed;
    // ctx.targetAngle = ctx.desiredAngle;
    return std::make_unique<StateTurning>();
  }
  //   ctx.droneAngle = ctx.desiredAngle;
  ctx.timeToStop = 0.f;
  return std::make_unique<StateAccelerating>();
}