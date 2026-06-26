#include "interfaces/IDroneState.h"
#include "states/StateMoving.h"
#include "Types.h"
#include "models/Coord.h"
#include "states/StateDecelerating.h"
#include "util.h"

#include <cmath>
#include <memory>

std::unique_ptr<IDroneState> StateMoving::execute(MissionContext &ctx)
{
  ctx.state = this->name();

  util::convergeAngle(ctx.droneAngle, ctx.targetAngle, ctx.cfg);

  float angleDelta = util::normalizeAngle(ctx.targetAngle - ctx.droneAngle);

  if (fabsf(angleDelta) > ctx.cfg.turnThreshold) {
    return std::make_unique<StateDecelerating>();
  }

  float ds = static_cast<float>(ctx.droneSpeed * ctx.cfg.simTimeStep);
  Coord dir = {static_cast<float>(cos(ctx.droneAngle)), static_cast<float>(sin(ctx.droneAngle))};
  ctx.dronePos = ctx.dronePos + dir * ds;
  return std::make_unique<StateMoving>();
}
