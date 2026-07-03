#include "states/StateAccelerating.h"
#include "Types.h"
#include "interfaces/IDroneState.h"
#include "states/StateDecelerating.h"
#include "states/StateMoving.h"
#include "util.h"

#include <cmath>
#include <memory>

std::unique_ptr<IDroneState> StateAccelerating::execute(MissionContext &ctx)
{
  ctx.commandMode = ACCELERATING;

  // Залишаємо час для прискорення, поки не досягнемо швидкості атаки або не перевищимо поріг повороту.
  // Фізика оновить дані сама
  float postAngle = ctx.droneAngle;
  util::convergeAngle(postAngle, ctx.targetAngle, ctx.cfg, ctx.cfg.simTimeStep);
  float angleDelta = util::normalizeAngle(ctx.targetAngle - postAngle);

  if (fabsf(angleDelta) > ctx.cfg.turnThreshold) {
    return std::make_unique<StateDecelerating>();
  }

  float postSpeed = ctx.droneSpeed + ctx.droneAccel * ctx.cfg.simTimeStep;

  ctx.timeToStop = (ctx.cfg.attackSpeed - postSpeed) / ctx.droneAccel;

  if (postSpeed >= ctx.cfg.attackSpeed) {
    return std::make_unique<StateMoving>();
  }
  return std::make_unique<StateAccelerating>();
}