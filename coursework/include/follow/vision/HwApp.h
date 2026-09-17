#pragma once

#include <atomic>
#include <filesystem>
#include <optional>
#include <ostream>
#include <string>

#include "follow/vision/FrameSource.h"

namespace follow::vision {

namespace detail {

// Whether the overlay should redraw at `now`, given the last scheduled draw instant `nextOverlay`
// and the configured `period`. When it returns true, `nextOverlay` is advanced by exactly one
// `period` -- not reset to `now` -- so a draw that runs late does not shorten the next gap (the fix
// for Important 5: restarting from the draw instant quantises overlay_fps to fps/2). If that leaves
// `nextOverlay` still behind `now` (more than one whole period behind), it is clamped forward to
// `now` so a long stall recovers instead of bursting through every missed slot.
bool shouldDrawOverlay(core::TimePoint now, core::TimePoint& nextOverlay, core::Clock::duration period);

}  // namespace detail

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
