#pragma once

namespace follow::config {
struct AppConfig;
}

namespace follow::interfaces {

// Where the application configuration comes from. `load()` does the reading and throws
// config::ConfigError on a bad source; `getConfig()` hands back what it read.
//
// Deliberately includes nothing: no JSON, no OpenCV, not even AppConfig's own header. The mission
// processors and ComponentFactory hold one of these, and both must stay buildable in the default
// FOLLOW_WITH_OPENCV=OFF tree. AppConfig is therefore only forward-declared -- a function
// declaration may return an incomplete type, and every implementation and caller sees the
// definition through config/FileConfigLoader.h anyway.
class IConfigLoader {
public:
  virtual ~IConfigLoader() = default;

  virtual void load() = 0;
  virtual config::AppConfig getConfig() = 0;
};

}  // namespace follow::interfaces
