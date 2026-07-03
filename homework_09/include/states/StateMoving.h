#pragma once
#include <memory>
#include "Types.h"
#include "interfaces/IDroneState.h"

class StateMoving : public IDroneState {
public:
  const char* name() const override { return "Moving"; }
  std::unique_ptr<IDroneState> execute(MissionContext& ctx) override;
};