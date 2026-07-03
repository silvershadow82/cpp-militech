#pragma once

#include "models/Target.h"

class ITargetProvider {
public:
  virtual int getTargetCount() const = 0;
  // Returns a Target BY VALUE on purpose. In the multithreaded design Target is a
  // lightweight snapshot (index + current pos + current velocity; the full
  // trajectory lives inside the provider), and the provider thread mutates the
  // backing target while the mission thread reads it. Copying under the provider's
  // mutex hands the caller a race-free snapshot. Returning `const Target&` here
  // would expose concurrently-mutated storage (data race) — do NOT change it.
  virtual Target getTarget(int index) const = 0;
  virtual ~ITargetProvider() = default;
};