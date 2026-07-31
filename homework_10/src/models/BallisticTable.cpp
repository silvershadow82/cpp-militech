#include "models/BallisticTable.h"

#include <algorithm>
#include <fstream>

// Завантаження з текстового файлу
bool BallisticTable::load(const char *path)
{
  std::ifstream f(path);
  if (!f.is_open())
    return false;

  int nZ, nV, nM, nD, nL;
  f >> nZ >> nV >> nM >> nD >> nL;

  axisZ0.resize(nZ);
  for (auto &v : axisZ0)
    f >> v;
  axisV0.resize(nV);
  for (auto &v : axisV0)
    f >> v;
  axisM.resize(nM);
  for (auto &v : axisM)
    f >> v;
  axisD.resize(nD);
  for (auto &v : axisD)
    f >> v;
  axisL.resize(nL);
  for (auto &v : axisL)
    f >> v;

  size_t total = (size_t)nZ * nV * nM * nD * nL;
  data.resize(total);

  // Порядок: Z0 → V0 → m → d → l (зовнішній → внутрішній)
  for (size_t i = 0; i < total; i++)
    f >> data[i].t >> data[i].hDist;

  return !f.fail();
}

// Лінійна інтерполяція для Result (обидва поля паралельно)
BallisticTable::Result BallisticTable::lerp(const Result &a, const Result &b, float t) const
{
  return {a.t + (b.t - a.t) * t, a.hDist + (b.hDist - a.hDist) * t};
}

BallisticTable::Interp BallisticTable::findInterp(float val, const std::vector<float> &axis) const
{
  if (val <= axis.front())
    return {0, 0.0f};
  if (val >= axis.back())
    return {(int)axis.size() - 2, 1.0f};

  auto it = std::lower_bound(axis.begin(), axis.end(), val);
  int i = (int)(it - axis.begin()) - 1;
  if (i < 0)
    i = 0;

  float frac = (val - axis[i]) / (axis[i + 1] - axis[i]);
  return {i, frac};
}

BallisticTable::Result BallisticTable::lookup(float Z0, float V0, float m, float d, float l) const
{
  Interp iz = findInterp(Z0, axisZ0);
  Interp iv = findInterp(V0, axisV0);
  Interp im = findInterp(m, axisM);
  Interp id = findInterp(d, axisD);
  Interp il = findInterp(l, axisL);

  // 2^5 = 32 вершини гіперкуба
  // Згортаємо: 32 → 16 → 8 → 4 → 2 → 1

  // l: 32 → 16
  Result v[16];
  for (int a = 0; a < 2; a++)
    for (int b = 0; b < 2; b++)
      for (int c = 0; c < 2; c++)
        for (int e = 0; e < 2; e++) {
          auto &lo = at(iz.lo + a, iv.lo + b, im.lo + c, id.lo + e, il.lo);
          auto &hi = at(iz.lo + a, iv.lo + b, im.lo + c, id.lo + e, il.lo + 1);
          v[a * 8 + b * 4 + c * 2 + e] = lerp(lo, hi, il.frac);
        }

  // d: 16 → 8
  Result w[8];
  for (int a = 0; a < 2; a++)
    for (int b = 0; b < 2; b++)
      for (int c = 0; c < 2; c++)
        w[a * 4 + b * 2 + c] = lerp(v[a * 8 + b * 4 + c * 2], v[a * 8 + b * 4 + c * 2 + 1], id.frac);

  // m: 8 → 4
  Result u[4];
  for (int a = 0; a < 2; a++)
    for (int b = 0; b < 2; b++)
      u[a * 2 + b] = lerp(w[a * 4 + b * 2], w[a * 4 + b * 2 + 1], im.frac);

  // V0: 4 → 2
  Result s[2];
  for (int a = 0; a < 2; a++)
    s[a] = lerp(u[a * 2], u[a * 2 + 1], iv.frac);

  // Z0: 2 → 1
  return lerp(s[0], s[1], iz.frac);
}
