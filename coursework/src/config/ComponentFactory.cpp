#include "config/ComponentFactory.h"

#include <memory>
#include <utility>

#include "comms/LinkSpec.h"

namespace follow::config {

std::unique_ptr<interfaces::IConfigLoader> ComponentFactory::createConfigLoader(const std::filesystem::path& path, nlohmann::json overrides)
{
  return std::make_unique<FileConfigLoader>(path, std::move(overrides));
}

std::unique_ptr<ScenarioLoader> ComponentFactory::createScenarioLoader(const std::filesystem::path& path)
{
  return std::make_unique<ScenarioLoader>(path);
}

std::unique_ptr<interfaces::ICameraModel> ComponentFactory::createCameraModel(const CameraSettings& settings)
{
  return makeCameraModel(settings);
}

std::unique_ptr<interfaces::IByteLink> ComponentFactory::createLink(const std::string& spec)
{
  return comms::openLink(comms::parseLinkSpec(spec));
}

}  // namespace follow::config
