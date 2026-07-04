#pragma once
#include "interfaces/ITargetProvider.h"
#include "models/Target.h"
#include <vector>

class JsonTargetProvider : public ITargetProvider {
private:
  std::string jsonFileName;
  std::vector<Target> targets;

  int readTargetData();

public:
  JsonTargetProvider(const std::string &jsonFileName);
  int getTargetCount() override;
  Target getTarget(int index) override;
  ~JsonTargetProvider() override;
};