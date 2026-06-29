#pragma once
#include <memory>
#include "Types.h"
#include "interfaces/IDroneState.h"

class StateStopped : public IDroneState {
public:
  const char* name() const override { return "Stopped"; }
  std::unique_ptr<IDroneState> execute(MissionContext& ctx) override;
};