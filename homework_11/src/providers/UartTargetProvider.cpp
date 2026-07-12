#include "providers/UartTargetProvider.h"

UartTargetProvider::UartTargetProvider(int nTargets, float timeScale)
    : timeScale(timeScale > 0.0f ? timeScale : 1.0f)
{
  int count = nTargets > 0 ? nTargets : 0;
  this->targets.resize(static_cast<size_t>(count));
  for (int i = 0; i < count; ++i)
  {
    this->targets[static_cast<size_t>(i)].target.index = i;
  }
}

void UartTargetProvider::update(const dlink::TargetPos &pos)
{
  if (pos.id >= this->targets.size())
  {
    return;
  }

  TrackedTarget &tracked = this->targets[pos.id];
  Coord newPos{pos.x, pos.y};
  auto now = std::chrono::steady_clock::now();

  if (tracked.seen)
  {
    float dt = std::chrono::duration<float>(now - tracked.lastUpdate).count();
    float dtSim = dt * this->timeScale;
    if (dtSim > 1e-3) // щоб коректно обчислювати швидкість, потрібен ненульовий інтервал часу
    {
      Coord rawVelocity = (newPos - tracked.target.pos) / dtSim;
      tracked.target.velocity = tracked.target.velocity + rawVelocity;
    }
  }
  else
  {
    tracked.seen = true;
    ++this->seenCount;
  }

  tracked.target.pos = newPos;
  tracked.lastUpdate = now;
}

bool UartTargetProvider::allSeen() const
{
  return this->seenCount >= static_cast<int>(this->targets.size());
}

int UartTargetProvider::getTargetCount() const
{
  return static_cast<int>(this->targets.size());
}

Target UartTargetProvider::getTarget(int index) const
{
  if (index >= 0 && index < static_cast<int>(this->targets.size()))
  {
    return this->targets[static_cast<size_t>(index)].target;
  }
  return Target{};
}
