#pragma once

#include "target.hpp"
#include <string>

class ITargetProvider {
public:
  virtual int getTargetCount() = 0;
  virtual Target getTarget(int index) = 0;
  virtual ~ITargetProvider(){};
};

class JsonTargetProvider : public ITargetProvider {
private:
  const std::string jsonFileName;
  int targetCount{0};
  Target* targets;

  int readTargetData();

public:
  JsonTargetProvider(const std::string& jsonFileName);
  int getTargetCount() override;
  Target getTarget(int index) override;
  ~JsonTargetProvider() override;
};