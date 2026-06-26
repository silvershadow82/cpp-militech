#include "states/StateAccelerating.h"
#include "Types.h"
#include "interfaces/IDroneState.h"
#include "states/StateDecelerating.h"
#include "states/StateMoving.h"
#include "util.h"

#include <cmath>
#include <memory>

std::unique_ptr<IDroneState> StateAccelerating::execute(DroneContext &ctx)
{
  ctx.state = this->name();

  util::convergeAngle(ctx.droneAngle, ctx.targetAngle, ctx.cfg);

  float angleDelta = util::normalizeAngle(ctx.targetAngle - ctx.droneAngle);

  if (fabsf(angleDelta) > ctx.cfg.turnThreshold) {
    return std::make_unique<StateDecelerating>();
  }

  float ds = static_cast<float>(ctx.droneSpeed * ctx.cfg.simTimeStep + 0.5f * ctx.droneAccel * ctx.cfg.simTimeStep * ctx.cfg.simTimeStep);
  Coord dir = {static_cast<float>(cos(ctx.droneAngle)), static_cast<float>(sin(ctx.droneAngle))};

  ctx.droneSpeed += ctx.droneAccel * ctx.cfg.simTimeStep;
  ctx.dronePos = ctx.dronePos + dir * ds;
  ctx.timeToStop = (ctx.cfg.attackSpeed - ctx.droneSpeed) / ctx.droneAccel;

  if (ctx.droneSpeed >= ctx.cfg.attackSpeed) {
    ctx.droneSpeed = ctx.cfg.attackSpeed;
    return std::make_unique<StateMoving>();
  }
  return std::make_unique<StateAccelerating>();
}