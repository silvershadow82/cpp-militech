#pragma once

#include <filesystem>
#include <memory>
#include <nlohmann/json.hpp>
#include <string>

#include "Types.h"
#include "config/FileConfigLoader.h"
#include "config/ScenarioLoader.h"
#include "interfaces/IByteLink.h"
#include "interfaces/ICameraModel.h"
#include "interfaces/IConfigLoader.h"

namespace follow::interfaces {
class IFrameSource;
class ITracker;
}  // namespace follow::interfaces

namespace follow::providers {
struct PiCameraConfig;
}

namespace follow::config {

// Builds the application's components, so the mission processors and main() never name a concrete
// class. Everything comes back as std::unique_ptr to an interface.
//
// The OpenCV-only half -- createTracker and the frame sources -- is declared here but defined in
// src/config/ComponentFactoryVision.cpp, which is compiled into follow_vision only. The interfaces
// it returns are forward-declared rather than included, because interfaces/IFrameSource.h and
// interfaces/ITracker.h both pull <opencv2/core.hpp> and this header is used by the default
// FOLLOW_WITH_OPENCV=OFF build. Nothing calls those methods in an OFF build, so they never have to
// link there; the class layout is the same either way, since no #ifdef touches it.
class ComponentFactory {
public:
  std::unique_ptr<interfaces::IConfigLoader> createConfigLoader(const std::filesystem::path& path,
                                                                nlohmann::json overrides = nlohmann::json::object());
  std::unique_ptr<ScenarioLoader> createScenarioLoader(const std::filesystem::path& path);
  std::unique_ptr<interfaces::ICameraModel> createCameraModel(const CameraSettings& settings);
  // `spec` is "udp:PORT", "udp:PORT:HOST:PORT" or "uart:DEVICE:BAUD"; see comms/LinkSpec.h.
  std::unique_ptr<interfaces::IByteLink> createLink(const std::string& spec);

  // OpenCV-only, follow_vision.
  std::unique_ptr<interfaces::ITracker> createTracker(const std::string& name);
  std::unique_ptr<interfaces::IFrameSource> createFrameSource(const providers::PiCameraConfig& config);
  std::unique_ptr<interfaces::IFrameSource> createFrameSource(
    const std::filesystem::path& video, int trackWidth, int trackHeight, double fps, models::TimePoint start);
  std::unique_ptr<interfaces::IFrameSource> createFrameSource(int width, int height, models::TimePoint start);
};

}  // namespace follow::config
