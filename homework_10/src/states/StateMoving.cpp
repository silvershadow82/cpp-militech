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
  ctx.commandMode = MOVING;

  // Decide-only: physics integrates; here we only read the post-converge angle
  // on a local copy to pick the next state.
  float postAngle = ctx.droneAngle;
  util::convergeAngle(postAngle, ctx.targetAngle, ctx.cfg, ctx.cfg.simTimeStep);

  float angleDelta = util::normalizeAngle(ctx.targetAngle - postAngle);

  if (fabsf(angleDelta) > ctx.cfg.turnThreshold) {
    return std::make_unique<StateDecelerating>();
  }

  return std::make_unique<StateMoving>();
}
