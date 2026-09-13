#include "follow/core/Core.h"

#include <algorithm>
#include <chrono>

namespace follow::core {

namespace {

bool isEngaged(State state)
{
  return state == State::Locking || state == State::Following || state == State::Lost || state == State::Hold;
}

}  // namespace

Core::Core(const Config& config, const CameraModel& camera, const CameraMount& mount)
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

  State before = this->supervisor.state();
  TargetState target = this->estimator.update(inputs.now, inputs.vehicle.attitude, inputs.target, inputs.range);
  State after = this->supervisor.update(inputs.now, inputs.vehicle.lastHeartbeat, inputs.vehicle.customMode, target.valid);

  Outputs out{};
  out.state = after;
  out.target = target;

  if (after != before) {
    switch (after) {
      case State::Locking:
        this->estimator.lock(inputs.now);
        this->controller.reset();
        out.tracker = {.kind = TrackerRequestKind::LockCenter, .hint = this->lockBox()};
        break;
      case State::Following:
        this->controller.reset();
        break;
      case State::Lost:
        if (std::optional<BBox> box = this->estimator.lastGoodBox()) {
          out.tracker = {.kind = TrackerRequestKind::Reacquire, .hint = *box};
        }
        break;
      case State::Idle:
      case State::NoFc:
        if (isEngaged(before)) {
          this->estimator.reset();
          this->controller.reset();
          out.tracker = {.kind = TrackerRequestKind::Unlock};
        }
        break;
      case State::Hold:
        break;
    }
  }

  switch (after) {
    case State::Following:
      out.setpoint = this->controller.update(target, dt);
      break;
    case State::Locking:
    case State::Lost:
    case State::Hold:
      out.setpoint = VelocityCmd{};
      break;
    case State::Idle:
    case State::NoFc:
      break;
  }

  out.overlay = {.state = after, .lockBox = this->lockBox(), .targetBox = target.valid ? this->estimator.lastGoodBox() : std::nullopt};
  return out;
}

BBox Core::lockBox() const
{
  const Intrinsics& k = this->camera.intrinsics();
  double side = this->config.lockBoxFrac * k.height;
  return {.x = (k.width - side) / 2.0, .y = (k.height - side) / 2.0, .w = side, .h = side};
}

}  // namespace follow::core
