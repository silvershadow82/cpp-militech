#pragma once

struct Coord {
  float x;
  float y;

  Coord operator+(const Coord &other) const {
    return Coord{x + other.x, y + other.y};
  }

  Coord operator-(const Coord &other) const {
    return Coord{x - other.x, y - other.y};
  }

  Coord operator*(float scalar) const { return Coord{x * scalar, y * scalar}; }

  Coord operator/(float scalar) const {
    if (scalar != 0) {
      return Coord{x / scalar, y / scalar};
    }
    return Coord{x, y};
  }

  bool operator==(const Coord &other) const {
    return x == other.x && y == other.y;
  }
  static inline Coord predict(Coord coord, Coord speed, float time) {
    return coord + speed * time;
  }

  // Deinitions yet to implement
  float length();
  Coord normalize();
  static float distance(Coord coord1, Coord coord2);
  static float angle(Coord coord1, Coord coord2);
};