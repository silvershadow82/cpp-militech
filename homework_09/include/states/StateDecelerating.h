#pragma once
#include <memory>
#include "Types.h"
#include "interfaces/IDroneState.h"

class StateDecelerating : public IDroneState {
public:
  const char* name() const override { return "Decelerating"; }
  std::unique_ptr<IDroneState> execute(DroneContext& ctx) override;
};