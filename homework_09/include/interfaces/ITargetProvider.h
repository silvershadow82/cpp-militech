#pragma once

#include "models/Target.h"

class ITargetProvider {
public:
  virtual int getTargetCount() = 0;
  virtual Target getTarget(int index) = 0;
};