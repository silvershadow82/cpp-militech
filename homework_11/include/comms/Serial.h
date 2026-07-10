#pragma once

#include <any>

class Serial {
private:
  const char* device;
  int fd;

public:
  int open(const char* device);
  void sendControl(float accel, float turnRate);
  std::any readFrame();
};