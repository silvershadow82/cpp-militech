#pragma once

#include <atomic>
#include <filesystem>
#include <optional>
#include <ostream>
#include <string>

#include "follow/vision/FrameSource.h"

namespace follow::vision {

struct HwAppOptions {
  std::filesystem::path configPath{"config/follow.json"};
  std::optional<std::string> link{};  // overrides mavlink.link from follow.json when set
  std::filesystem::path logPath{"follow_run.csv"};
};

// follow_app --hw: loads follow.json and runs the MAVLink I/O, camera + tracker and control threads
// until `stop` is set or the frame source ends. The overlay goes to vision.framebuffer when it opens;
// a missing framebuffer is reported on `out` and the app runs without an overlay.
// Throws config::ConfigError or std::runtime_error for bad files, links or log paths.
void runHwApp(const HwAppOptions& options, IFrameSource& frames, const std::atomic<bool>& stop, std::ostream& out);

}  // namespace follow::vision
