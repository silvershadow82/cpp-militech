#include "config/ComponentFactory.h"
#include "interfaces/IFrameSource.h"
#include "interfaces/ITracker.h"
#include "providers/PiCameraSource.h"
#include "providers/SyntheticFrameSource.h"
#include "providers/VideoFileSource.h"
#include "vision/TrackerFactory.h"

#include <memory>

namespace follow::config {

std::unique_ptr<interfaces::ITracker> ComponentFactory::createTracker(const std::string &name)
{
  return vision::makeTracker(name);
}

std::unique_ptr<interfaces::IFrameSource> ComponentFactory::createFrameSource(const providers::PiCameraConfig &config)
{
  return std::make_unique<providers::PiCameraSource>(config);
}

std::unique_ptr<interfaces::IFrameSource> ComponentFactory::createFrameSource(
  const std::filesystem::path &video, int trackWidth, int trackHeight, double fps, models::TimePoint start)
{
  return std::make_unique<providers::VideoFileSource>(video, cv::Size(trackWidth, trackHeight), fps, start);
}

std::unique_ptr<interfaces::IFrameSource> ComponentFactory::createFrameSource(int width, int height, models::TimePoint start)
{
  return std::make_unique<providers::SyntheticFrameSource>(width, height, start);
}

}  // namespace follow::config
