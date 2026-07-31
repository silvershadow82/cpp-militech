#include "interfaces/IDroneState.h"
#include "states/StateMoving.h"
#include "Types.h"
#include "states/StateDecelerating.h"
#include "util.h"

#include <cmath>
#include <memory>

std::unique_ptr<IDroneState> StateMoving::execute(MissionContext &ctx)
{
  ctx.commandMode = MOVING;

  float postAngle = ctx.droneAngle;

  util::convergeAngle(postAngle, ctx.targetAngle, ctx.cfg, ctx.cfg.simTimeStep);

  float angleDelta = util::normalizeAngle(ctx.targetAngle - postAngle);

  if (fabsf(angleDelta) > ctx.cfg.turnThreshold) {
    return std::make_unique<StateDecelerating>();
  }

  return std::make_unique<StateMoving>();
}
