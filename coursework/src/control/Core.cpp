#include "control/Core.h"

#include <algorithm>
#include <chrono>

namespace follow::control {

namespace {

bool isEngaged(models::State state)
{
  return state == models::State::Locking || state == models::State::Following || state == models::State::Lost ||
         state == models::State::Hold;
}

}  // namespace

Core::Core(const models::Config& config, const interfaces::ICameraModel& camera, const models::CameraMount& mount)
  : config(config)
  , camera(camera)
  , estimator(this->config.estimator, camera, mount)
  , controller(this->config.control)
  , supervisor(this->config.supervisor)
{
}

Outputs Core::step(const Inputs& inputs)
{
  double dt = this->lastStep ? std::chrono::duration<double>(inputs.now - *this->lastStep).count() : 1.0 / this->config.rateHz;
  dt = std::clamp(dt, 0.0, 0.2);
  this->lastStep = inputs.now;

  models::State before = this->supervisor.state();
  models::TargetState target = this->estimator.update(inputs.now, inputs.vehicle.attitude, inputs.target, inputs.range);
  models::State after = this->supervisor.update(inputs.now, inputs.vehicle.lastHeartbeat, inputs.vehicle.customMode, target.valid);

  Outputs out{};
  out.state = after;
  out.target = target;

  if (after != before) {
    switch (after) {
      case models::State::Locking:
        this->estimator.lock(inputs.now);
        this->controller.reset();
        out.tracker = {.kind = TrackerRequestKind::LockCenter, .hint = this->lockBox()};
        break;
      case models::State::Following:
        this->controller.reset();
        break;
      case models::State::Lost:
        if (std::optional<models::BBox> box = this->estimator.lastGoodBox()) {
          out.tracker = {.kind = TrackerRequestKind::Reacquire, .hint = *box};
        }
        break;
      case models::State::Idle:
      case models::State::NoFc:
        if (isEngaged(before)) {
          this->estimator.reset();
          this->controller.reset();
          out.tracker = {.kind = TrackerRequestKind::Unlock};
        }
        break;
      case models::State::Hold:
        break;
    }
  }

  switch (after) {
    case models::State::Following:
      out.setpoint = this->controller.update(target, dt);
      break;
    case models::State::Locking:
    case models::State::Lost:
    case models::State::Hold:
      out.setpoint = models::VelocityCmd{};
      break;
    case models::State::Idle:
    case models::State::NoFc:
      break;
  }

  out.overlay = {.state = after, .lockBox = this->lockBox(), .targetBox = target.valid ? this->estimator.lastGoodBox() : std::nullopt};
  return out;
}

models::BBox Core::lockBox() const
{
  const models::Intrinsics& k = this->camera.intrinsics();
  double side = this->config.lockBoxFrac * k.height;
  return {.x = (k.width - side) / 2.0, .y = (k.height - side) / 2.0, .w = side, .h = side};
}

}  // namespace follow::control
