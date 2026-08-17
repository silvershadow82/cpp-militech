#include "Types.h"
#include "interfaces/IDroneState.h"
#include "states/StateDecelerating.h"
#include "states/StateStopped.h"

#include <cmath>
#include <memory>

std::unique_ptr<IDroneState> StateDecelerating::execute(MissionContext &ctx)
{
  ctx.commandMode = DECELERATING;

  // Сповільнення дрона до зупинки
  float ds = static_cast<float>(ctx.droneSpeed * ctx.cfg.simTimeStep - 0.5f * ctx.droneAccel * ctx.cfg.simTimeStep * ctx.cfg.simTimeStep);
  float postSpeed = ctx.droneSpeed - ctx.droneAccel * ctx.cfg.simTimeStep;

  if (postSpeed <= 0.f) {
    return std::make_unique<StateStopped>();
  }
  if (ds > 0.f) {
    ctx.timeToStop = postSpeed / ctx.droneAccel;
  }

  return std::make_unique<StateDecelerating>();
}