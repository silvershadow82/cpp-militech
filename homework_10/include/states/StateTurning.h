#pragma once
#include <memory>
#include "Types.h"
#include "interfaces/IDroneState.h"

class StateTurning : public IDroneState {
public:
  const char* name() const override { return "Turning"; }
  std::unique_ptr<IDroneState> execute(MissionContext& ctx) override;
};