#pragma once
#include <memory>
#include "Types.h"
#include "interfaces/IDroneState.h"
#include "states/StateMoving.h"

class StateTurning : public IDroneState {
public:
  const char* name() const override { return "Turning"; }
  std::unique_ptr<IDroneState> execute(DroneContext& ctx) override { return std::make_unique<StateMoving>(); }
};