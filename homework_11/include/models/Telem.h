#pragma once

#include "models/Coord.h"
#include <cstdint>

struct Telem
{
  Coord pos{};
  float altitude{0.f};
  float speed{0.f};
  float angle{0.f};
  uint32_t t_ms{0};
};
