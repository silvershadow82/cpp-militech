#pragma once

#include "interfaces/ITargetProvider.h"
#include "models/Coord.h"
#include "models/Target.h"
#include "models/drone_link.h"
#include <cstdint>
#include <vector>

class UartTargetProvider : public ITargetProvider
{
private:
  struct TrackedTarget
  {
    Target target{};
    bool seen{false};
    uint32_t lastUpdateMs{0};
  };

  std::vector<TrackedTarget> targets;
  int seenCount{0};

public:
  explicit UartTargetProvider(int nTargets);
  ~UartTargetProvider() override = default;

  // Оновлює стан цілі з отриманого кадру PKT_TARGET. Якщо це новий id, додає його в список.
  // Якщо це вже відомий id, оновлює його позицію і швидкість.
  // simTimeMs — час місії (t_ms з останньої отриманої телеметрії): швидкість цілі рахується
  // за симульованим часом чекера, а не wall-clock часом прийому кадру, який спотворюється
  // затримками читання з UART (напр. пачкова обробка кількох кадрів одразу дає майже нульовий
  // виміряний dt і завищену швидкість).
  void update(const dlink::TargetPos &pos, uint32_t simTimeMs);

  bool allSeen() const;

  int getTargetCount() const override;
  Target getTarget(int index) const override;
};
