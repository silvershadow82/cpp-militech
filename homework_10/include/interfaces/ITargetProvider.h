#pragma once

#include "models/Target.h"

class ITargetProvider {
public:
  virtual int getTargetCount() const = 0;
  virtual Target getTarget(int index) const = 0;
  virtual ~ITargetProvider() = default;
};