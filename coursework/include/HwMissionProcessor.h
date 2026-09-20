#pragma once

#include <atomic>
#include <filesystem>
#include <memory>
#include <optional>
#include <ostream>
#include <string>

#include "interfaces/IByteLink.h"
#include "interfaces/ICameraModel.h"
#include "interfaces/IConfigLoader.h"
#include "interfaces/IFrameSource.h"
#include "interfaces/ITracker.h"

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

// follow_app --hw: runs the MAVLink I/O, camera + tracker and control threads until `stop` is set or
// the frame source ends. The overlay goes to vision.framebuffer when it opens; a missing framebuffer
// is reported on `out` and the app runs without an overlay.
//
// The dependencies are built by config::ComponentFactory and injected here; the config loader must
// already have been load()ed, because the link and the camera model are derived from what it read.
// `frames` must outlive this object. run() throws std::runtime_error for an unwritable log path or
// a worker thread that failed -- and in the latter case only after the fail-safe has gone out.
class HwMissionProcessor {
public:
  HwMissionProcessor(HwAppOptions options,
                     std::unique_ptr<interfaces::IConfigLoader> configLoader,
                     std::unique_ptr<interfaces::IByteLink> link,
                     std::unique_ptr<interfaces::ICameraModel> camera,
                     std::unique_ptr<interfaces::ITracker> tracker,
                     interfaces::IFrameSource& frames);
  ~HwMissionProcessor();

  void run(const std::atomic<bool>& stop, std::ostream& out);

private:
  HwAppOptions options;
  std::unique_ptr<interfaces::IConfigLoader> configLoader;
  std::unique_ptr<interfaces::IByteLink> link;
  // Outlives the Core inside ControlLoop, which keeps a reference.
  std::unique_ptr<interfaces::ICameraModel> camera;
  // Moved into CameraTrackerSource by run(), which owns it for the rest of the run.
  std::unique_ptr<interfaces::ITracker> tracker;
  interfaces::IFrameSource& frames;
};

}  // namespace follow::app
