#pragma once

#include <filesystem>
#include <memory>
#include <nlohmann/json_fwd.hpp>
#include <stdexcept>
#include <string>

#include "interfaces/ICameraModel.h"
#include "models/Intrinsics.h"
#include "models/Config.h"
#include "models/Frames.h"

namespace follow::config {

// Invalid configuration. The message names the offending key, e.g. "control.k_d: ...".
class ConfigError : public std::runtime_error {
public:
  using std::runtime_error::runtime_error;
};

struct MavlinkSettings {
  std::string link{"uart:/dev/serial0:921600"};
  int sysid{1};     // same system as the FC
  int compid{191};  // MAV_COMP_ID_ONBOARD_COMPUTER
};

enum class CameraKind { Pinhole, Fisheye };

struct CameraSettings {
  CameraKind kind{CameraKind::Fisheye};
  models::Intrinsics intrinsics{models::nominalFisheye(640, 480, 160.0)};  // at the tracking resolution
  models::CameraMount mount{};
};

struct VisionSettings {
  int captureWidth{1640};
  int captureHeight{1232};
  int trackWidth{640};
  int trackHeight{480};
  std::string tracker{"kcf"};           // "kcf" or "csrt"
  int fps{20};                          // camera frame rate requested from libcamerasrc
  std::string framebuffer{"/dev/fb0"};  // overlay output; empty disables the overlay
  int overlayFps{15};
  int reacquirePeriodMs{500};
  double reacquireExpand{1.5};
  bool hflip{false};  // camera mounted mirrored left/right; flipped at capture, not in core
  bool vflip{false};  // camera mounted upside-down; flipped at capture, not in core
};

struct AppConfig {
  models::Config core{};
  MavlinkSettings mavlink{};
  VisionSettings vision{};
  CameraSettings camera{};
};

// follow.json without its camera file. Missing keys keep their defaults; unknown keys, wrong types
// and out-of-range values throw ConfigError.
AppConfig parseAppConfig(const nlohmann::json& doc);

// Camera file in OpenCV's parameter layout; intrinsics are scaled to the tracking resolution.
CameraSettings parseCamera(const nlohmann::json& doc, int trackWidth, int trackHeight);

// Reads a JSON file; ConfigError if it cannot be opened or parsed.
nlohmann::json readJsonFile(const std::filesystem::path& path);

// Reads follow.json plus the camera file it names, relative to follow.json's directory.
AppConfig loadAppConfig(const std::filesystem::path& path);
// Same, with `overrides` applied to follow.json as a JSON merge patch (RFC 7386) before parsing.
AppConfig loadAppConfig(const std::filesystem::path& path, const nlohmann::json& overrides);

std::unique_ptr<interfaces::ICameraModel> makeCameraModel(const CameraSettings& settings);

}  // namespace follow::config
