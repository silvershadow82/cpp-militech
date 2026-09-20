#pragma once

#include <atomic>
#include <filesystem>
#include <optional>
#include <ostream>
#include <string>

#include "interfaces/IFrameSource.h"

namespace follow::app {

namespace detail {

// Whether the overlay should redraw at `now`, given the last scheduled draw instant `nextOverlay`
// and the configured `period`. When it returns true, `nextOverlay` is advanced by exactly one
// `period` -- not reset to `now` -- so a draw that runs late does not shorten the next gap (the fix
// for Important 5: restarting from the draw instant quantises overlay_fps to fps/2). If that leaves
// `nextOverlay` still behind `now` (more than one whole period behind), it is clamped forward to
// `now + period` -- not `now` alone, which would make the very next tick immediately eligible again,
// a back-to-back double draw -- so a long stall recovers instead of bursting through every missed slot.
bool shouldDrawOverlay(models::TimePoint now, models::TimePoint& nextOverlay, models::Clock::duration period);

// Whether a camera-missing/recovered edge should actually print, given the last time such a line was
// printed (`lastReport`, or nullopt if none has printed yet) and the minimum gap `minInterval`
// required between two lines. Pure: the caller updates `lastReport` itself when this returns true.
// Exists because edge-triggering alone (print on missing, print again on recovered) still reports on
// every transition -- for an alternating drop/good pattern that is a line every frame, forever, not
// just for a multi-frame burst -- so this bounds total output to at most one line per `minInterval`
// regardless of the drop pattern.
bool shouldReportMissEdge(models::TimePoint now, std::optional<models::TimePoint> lastReport, models::Clock::duration minInterval);

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
void runHwApp(const HwAppOptions& options, interfaces::IFrameSource& frames, const std::atomic<bool>& stop, std::ostream& out);

}  // namespace follow::app
