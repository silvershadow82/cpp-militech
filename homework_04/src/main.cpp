#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>

const int ticker_per_revolution = 1024;
const float wheel_radius_m = 0.3;
const float wheel_base_m = 1.0;

int main(int argc, char** argv)
{
  // The program expects exactly one argument: a path to telemetry samples.
  if (argc != 2) {
    std::cerr << "usage: ugv_odometry <input_path>\n";
    return 1;
  }

  const char* inputFileName{argv[1]};

  std::ifstream inputFile(inputFileName);

  if (!inputFile.is_open()) {
    std::cerr << "Unable to open input file: " << inputFileName << std::endl;
    return 1;
  }

  long timestamp_ms{0}, fl_ticks{0}, fr_ticks{0}, bl_ticks{0}, br_ticks{0}, prev_fl_ticks{0}, prev_fr_ticks{0}, prev_bl_ticks{0},
    prev_br_ticks{0};
  float d_fl{0.f}, d_fr{0.f}, d_bl{0.f}, d_br{0.f}, d_left{0.f}, d_right{0.f}, distance_per_tick{0.f}, dL{0.f}, dR{0.f}, d{0.f},
    dthetha{0.f}, x{0.f}, y{0.f}, theta{0.f};

  // Now main cycle
  for (int i = 0; !inputFile.eof(); i++) {
    inputFile >> timestamp_ms >> fl_ticks >> fr_ticks >> bl_ticks >> br_ticks;

    if (i == 0) {
      prev_fl_ticks = fl_ticks;
      prev_fr_ticks = fr_ticks;
      prev_bl_ticks = bl_ticks;
      prev_br_ticks = br_ticks;
      continue;
    }

    d_fl = fl_ticks - prev_fl_ticks;
    d_fr = fr_ticks - prev_fr_ticks;
    d_bl = bl_ticks - prev_bl_ticks;
    d_br = br_ticks - prev_br_ticks;

    d_left = (d_fl + d_bl) / 2;
    d_right = (d_fr + d_br) / 2;

    distance_per_tick = 2 * M_PI * wheel_radius_m / ticker_per_revolution;

    dL = d_left * distance_per_tick;
    dR = d_right * distance_per_tick;

    d = (dL + dR) / 2;
    dthetha = (dR - dL) / wheel_base_m;

    x += d * cos(theta + dthetha / 2);
    y += d * sin(theta + dthetha / 2);

    // Output data
    std::cout << timestamp_ms << " " << x << " " << y << " " << theta << std::endl;

    prev_fl_ticks = fl_ticks;
    prev_fr_ticks = fr_ticks;
    prev_bl_ticks = bl_ticks;
    prev_br_ticks = br_ticks;

    theta += dthetha;
  }

  // Close the file
  inputFile.close();

  return 0;
}
