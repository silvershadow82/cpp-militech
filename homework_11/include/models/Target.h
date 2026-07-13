#pragma once

#include "models/Coord.h"

struct Target {
  int index;       // індекс цілі
  Coord pos;       // поточна позиція цілі
  Coord velocity;  // поточна швидкість цілі
};