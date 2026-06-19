#pragma once

#include "Types.h"
#include <memory>

class IDroneState {
public:
  virtual ~IDroneState() = default;
  virtual std::unique_ptr<IDroneState> execute(DroneContext& ctx) = 0;
  virtual const char* name() const = 0;
};