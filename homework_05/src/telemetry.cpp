#include "telemetry.hpp"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <ostream>

// Debugging exercise notes:
// this file intentionally contains four runtime defects.
// The defects are related to malformed input shape, invalid numeric values,
// unsafe time deltas, and empty logs. Exact locations are not marked on purpose.

const int EXPECTED_FIELD_COUNT = 7;
const int MAX_LINE_LENGTH = 256;

int split_line(char line[], char* fields[], int max_fields)
{
  int count = 0;
  char* cursor = line;

  while (*cursor != '\0' && count < max_fields) {
    while (*cursor == ' ' || *cursor == '\t' || *cursor == '\n' || *cursor == '\r') {
      *cursor = '\0';
      ++cursor;
    }

    if (*cursor == '\0') {
      break;
    }

    fields[count] = cursor;
    ++count;

    while (*cursor != '\0' && *cursor != ' ' && *cursor != '\t' && *cursor != '\n' && *cursor != '\r') {
      ++cursor;
    }
  }

  return count;
}

long parse_long(const char* text)
{
  char* end = nullptr;
  const long value = std::strtol(text, &end, 10);

  if (end == text) {
    std::cerr << "error: unable to parse \"" << text << "\" as long" << std::endl;
    return static_cast<long>(-INFINITY);
  }

  return value;
}

int parse_int(const char* text)
{
  return static_cast<int>(parse_long(text));
}

double parse_double(const char* text)
{
  char* end = nullptr;
  const double value = std::strtod(text, &end);

  if (end == text) {
    std::cerr << "error: unable to parse \"" << text << "\" as double" << std::endl;
    return static_cast<double>(-INFINITY);
  }

  return value;
}

// Validate individual frame
bool intra_validate_frame(const Frame& frame)
{
  if (frame.voltage_v <= 0) {
    return false;
  }
  if (frame.temperature_c < -40.0 || frame.temperature_c > 120.0) {
    return false;
  }
  if (!(frame.gps_fix == 0 || frame.gps_fix == 1)) {
    return false;
  }
  if (frame.satellites < 0) {
    return false;
  }
  return true;
}

bool inter_validate_frames(const Frame frames[], int frame_count)
{
  if (frame_count < 2) {
    return true;
  }
  for (int i = 1; i < frame_count; i++) {
    Frame prevFrame = frames[i - 1];
    Frame currFrame = frames[i];
    if (currFrame.timestamp_ms <= prevFrame.timestamp_ms) {
      return false;
    }
    if ((currFrame.seq - prevFrame.seq) != 1) {
      return false;
    }
  }
  return true;
}

bool valid_long(long value)
{
  return value != -INFINITY;
}

bool valid_int(int value)
{
  return valid_long(value);
}

bool valid_double(double value)
{
  return value != -INFINITY;
}

Frame parse_frame(char line[], int line_count)
{
  char* fields[EXPECTED_FIELD_COUNT] = {};
  const int field_count = split_line(line, fields, EXPECTED_FIELD_COUNT);

  if (field_count != EXPECTED_FIELD_COUNT) {
    std::cerr << "error: invalid frame at line " << line_count << ": expected " << EXPECTED_FIELD_COUNT << " fields" << std::endl;
    return Frame{};
  }

  Frame frame{};
  frame.timestamp_ms = parse_long(fields[0]);
  if (!valid_long(frame.timestamp_ms)) {
    return Frame{};
  }
  frame.seq = parse_int(fields[1]);
  if (!valid_int(frame.seq)) {
    return Frame{};
  }
  frame.voltage_v = parse_double(fields[2]);
  if (!valid_double(frame.voltage_v)) {
    return Frame{};
  }
  frame.current_a = parse_double(fields[3]);
  if (!valid_double(frame.current_a)) {
    return Frame{};
  }
  frame.temperature_c = parse_double(fields[4]);
  if (!valid_double(frame.temperature_c)) {
    return Frame{};
  }
  frame.gps_fix = parse_int(fields[5]);
  if (!valid_int(frame.gps_fix)) {
    return Frame{};
  }
  frame.satellites = parse_int(fields[6]);
  if (!valid_int(frame.satellites)) {
    return Frame{};
  }

  if (!intra_validate_frame(frame)) {
    std::cerr << "error: invalid frame at line " << line_count << ": detected invalid field values" << std::endl;
    return Frame{};
  }

  return frame;
}

double compute_frame_rate_hz(const Frame frames[], int frame_count)
{
  const long elapsed_ms = frames[frame_count - 1].timestamp_ms - frames[0].timestamp_ms;

  return static_cast<double>((frame_count - 1) * 1000 / elapsed_ms);
}

int read_frames(const char* path, Frame frames[], int max_frames)
{
  std::ifstream input{path};
  if (!input) {
    std::cerr << "error: failed to open input file: " << path << '\n';
    return 0;
  }

  int frame_count = 0;
  char line[MAX_LINE_LENGTH];

  while (input.getline(line, MAX_LINE_LENGTH)) {
    if (line[0] == '\0') {
      continue;
    }

    if (frame_count < max_frames) {
      Frame frame = parse_frame(line, frame_count + 1);
      if (frame.seq == 0) {
        // не розпарсили фрейм, виходимо
        return 0;
      }
      frames[frame_count] = frame;
      ++frame_count;
    }
  }

  if (!inter_validate_frames(frames, frame_count)) {
    std::cerr << "error: invalid inter-frame sequence" << std::endl;
    return 0;
  }

  return frame_count;
}

Summary summarize(const Frame frames[], int frame_count)
{
  Summary summary{};
  summary.frames_total = frame_count;
  summary.frames_valid = frame_count;
  summary.voltage_min = frames[0].voltage_v;
  summary.voltage_max = frames[0].voltage_v;
  summary.low_voltage_frames = 0;

  double temperature_sum = 0.0;

  for (int i = 0; i < frame_count; ++i) {
    if (frames[i].voltage_v < summary.voltage_min) {
      summary.voltage_min = frames[i].voltage_v;
    }

    if (frames[i].voltage_v > summary.voltage_max) {
      summary.voltage_max = frames[i].voltage_v;
    }

    temperature_sum += frames[i].temperature_c;

    if (frames[i].voltage_v < 22.0) {
      ++summary.low_voltage_frames;
    }
  }

  const int temperature_tenths = static_cast<int>(temperature_sum * 10.0) / frame_count;
  summary.temperature_avg = static_cast<double>(temperature_tenths) / 10.0;
  summary.frame_rate_hz = compute_frame_rate_hz(frames, frame_count);
  return summary;
}

void print_summary(const Summary& summary)
{
  std::cout << "frames_total " << summary.frames_total << '\n';
  std::cout << "frames_valid " << summary.frames_valid << '\n';
  std::cout << "voltage_min " << summary.voltage_min << '\n';
  std::cout << "voltage_max " << summary.voltage_max << '\n';
  std::cout << "temperature_avg " << summary.temperature_avg << '\n';
  std::cout << "low_voltage_frames " << summary.low_voltage_frames << '\n';
  std::cout << "frame_rate_hz " << summary.frame_rate_hz << '\n';
}
