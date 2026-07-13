#include "providers/UartTargetProvider.h"

UartTargetProvider::UartTargetProvider(int nTargets)
{
  int count = nTargets > 0 ? nTargets : 0;
  this->targets.resize(static_cast<size_t>(count));
  for (int i = 0; i < count; ++i)
  {
    this->targets[static_cast<size_t>(i)].target.index = i;
  }
}

void UartTargetProvider::update(const dlink::TargetPos &pos, uint32_t simTimeMs)
{
  if (pos.id >= this->targets.size())
  {
    return;
  }

  TrackedTarget &tracked = this->targets[pos.id];
  Coord newPos{pos.x, pos.y};

  if (tracked.seen)
  {
    float dtSim = static_cast<float>(simTimeMs - tracked.lastUpdateMs) / 1000.0f;
    if (dtSim > 1e-3f) // щоб коректно обчислювати швидкість, потрібен ненульовий інтервал часу
    {
      tracked.target.velocity = (newPos - tracked.target.pos) / dtSim;
    }
  }
  else
  {
    tracked.seen = true;
    ++this->seenCount;
  }

  tracked.target.pos = newPos;
  tracked.lastUpdateMs = simTimeMs;
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
