#pragma once

#include "interfaces/ITargetProvider.h"
#include "models/Coord.h"
#include "models/Target.h"
#include "models/drone_link.h"
#include <chrono>
#include <cstdint>
#include <vector>

class UartTargetProvider : public ITargetProvider
{
private:
  struct TrackedTarget
  {
    Target target{};
    bool seen{false};
    std::chrono::steady_clock::time_point lastUpdate{};
  };

  std::vector<TrackedTarget> targets;
  int seenCount{0};
  float timeScale{1.0f};

public:
  explicit UartTargetProvider(int nTargets, float timeScale = 1.0f);
  ~UartTargetProvider() override = default;

  // Оновлює стан цілі з отриманого кадру PKT_TARGET. Якщо це новий id, додає його в список.
  // Якщо це вже відомий id, оновлює його позицію і швидкість.
  void update(const dlink::TargetPos &pos);

  bool allSeen() const;

  int getTargetCount() const override;
  Target getTarget(int index) const override;
};
