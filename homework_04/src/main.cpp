#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <ostream>
#include <string>

const int ticker_per_revolution = 1024;
const float wheel_radius_m = 0.3;
const float wheel_base_m = 1.0;

struct WheelMetrics {
  long timestamp_ms;
  long fl_ticks;
  long fr_ticks;
  long bl_ticks;
  long br_ticks;
};

int lineCount(const char* fileName)
{
  std::ifstream input(fileName);
  if (!input.is_open()) {
    return -1;
  }
  std::string line;
  int lineCount{0};

  while (std::getline(input, line)) {
    lineCount++;
  }

  input.close();
  return lineCount;
}

int main(int argc, char** argv)
{
  // The program expects exactly one argument: a path to telemetry samples.
  if (argc != 2) {
    std::cerr << "usage: ugv_odometry <input_path>\n";
    return 1;
  }

  const char* inputFileName{argv[1]};
  const int lines{lineCount(inputFileName)};

  // std::cout << "Counted " << lines << " lines in " << inputFileName << std::endl;

  // Reopen file for actual reading
  std::ifstream inputFile(inputFileName);

  if (!inputFile.is_open()) {
    std::cerr << "Unable to open input file: " << inputFileName << std::endl;
    return 1;
  }

  if (lines <= 0) {
    std::cerr << "Invalid input file data!" << std::endl;
    return 1;
  }

  // Read into struct
  WheelMetrics* metrics = new WheelMetrics[lines];

  for (int i = 0; i < lines; i++) {
    metrics[i] = WheelMetrics{};
    inputFile >> metrics[i].timestamp_ms >> metrics[i].fl_ticks >> metrics[i].fr_ticks >> metrics[i].bl_ticks >> metrics[i].br_ticks;
  }

  // Close early
  inputFile.close();

  float d_fl{0.f}, d_fr{0.f}, d_bl{0.f}, d_br{0.f}, d_left{0.f}, d_right{0.f}, distance_per_tick{0.f}, dL{0.f}, dR{0.f}, d{0.f},
    dthetha{0.f}, x{0.f}, y{0.f}, theta{0.f};

  // Now main cycle
  for (int i = 1; i < lines; i++) {
    d_fl = metrics[i].fl_ticks - metrics[i - 1].fl_ticks;
    d_fr = metrics[i].fr_ticks - metrics[i - 1].fr_ticks;
    d_bl = metrics[i].bl_ticks - metrics[i - 1].bl_ticks;
    d_br = metrics[i].br_ticks - metrics[i - 1].br_ticks;

    d_left = (d_fl + d_bl) / 2;
    d_right = (d_fr + d_br) / 2;
    distance_per_tick = 2 * M_PI * wheel_radius_m / ticker_per_revolution;

    dL = d_left * distance_per_tick;
    dR = d_right * distance_per_tick;

    d = (dL + dR) / 2;
    dthetha = (dR - dL) / wheel_base_m;

    x += d * cos(theta + dthetha / 2);
    y += d * sin(theta + dthetha / 2);

    theta += dthetha;
    // Output data
    std::cout << metrics[i].timestamp_ms << " " << x << " " << y << " " << theta << std::endl;
  }

  // Cleanup
  delete[] metrics;

  return 0;
}
