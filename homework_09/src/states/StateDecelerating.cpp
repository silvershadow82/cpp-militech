#include "Types.h"
#include "interfaces/IDroneState.h"
#include "states/StateDecelerating.h"
#include "states/StateStopped.h"
#include "util.h"

#include <cmath>
#include <memory>

std::unique_ptr<IDroneState> StateDecelerating::execute(MissionContext &ctx)
{
  ctx.state = this->name();

  util::convergeAngle(ctx.droneAngle, ctx.targetAngle, ctx.cfg);
  float ds = static_cast<float>(ctx.droneSpeed * ctx.cfg.simTimeStep - 0.5f * ctx.droneAccel * ctx.cfg.simTimeStep * ctx.cfg.simTimeStep);
  ctx.droneSpeed -= ctx.droneAccel * ctx.cfg.simTimeStep;

  if (ctx.droneSpeed <= 0.f) {
    ctx.droneSpeed = 0.f;
    return std::make_unique<StateStopped>();
  }
  if (ds > 0.f) {
    Coord dir = {static_cast<float>(cos(ctx.droneAngle)), static_cast<float>(sin(ctx.droneAngle))};
    ctx.dronePos = ctx.dronePos + dir * ds;
    ctx.timeToStop = ctx.droneSpeed / ctx.droneAccel;
  }

  return std::make_unique<StateDecelerating>();
}