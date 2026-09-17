#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <unistd.h>
#include <vector>

#include "FakeAutopilot.h"
#include "follow/config/ConfigJson.h"
#include "follow/core/Angles.h"
#include "follow/core/CameraModel.h"
#include "follow/core/Frames.h"
#include "follow/core/Types.h"
#include "follow/runtime/RunLog.h"
#include "follow/vision/FrameSource.h"
#include "follow/vision/HwApp.h"

namespace {

// Removes the temp directory on scope exit, including an uncaught exception, so a real failure
// (a bad run log, a config or link error) does not leak the directory.
struct TempDirGuard {
  explicit TempDirGuard(std::filesystem::path dir)
    : dir(std::move(dir))
  {
  }

  ~TempDirGuard()
  {
    std::error_code ec;
    std::filesystem::remove_all(this->dir, ec);
  }

  std::filesystem::path dir;
};

// Sets `stop` after `after`, or as soon as it is destroyed. runHwApp throws for a bad config, an
// unopenable link, an unwritable log and now a failing worker thread, and the stack then unwinds
// through this object: a std::thread destructing while still joinable calls std::terminate(), which
// aborts the whole binary before googletest can attribute the failure to a test. Waiting on a
// condition variable rather than sleeping also keeps the teardown immediate.
class Stopper {
public:
  Stopper(std::atomic<bool>& stop, std::chrono::milliseconds after)
    : stop(stop)
    , thread([this, after] {
      std::unique_lock<std::mutex> lock(this->mutex);
      this->cancelled.wait_for(lock, after, [this] { return this->done; });
      this->stop = true;
    })
  {
  }

  ~Stopper()
  {
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      this->done = true;
    }
    this->cancelled.notify_all();
    this->thread.join();
  }

private:
  std::atomic<bool>& stop;
  std::mutex mutex;
  std::condition_variable cancelled;
  bool done{false};
  std::thread thread;
};

// SyntheticFrameSource advances a virtual clock by exactly 50 ms per read() however long the
// iteration really took, and the vision loop gives up its schedule when it overruns
// (nextFrame = max(nextFrame + framePeriod, now)). The 50 ms the virtual clock did not advance is
// then lost for good, so `now - tFrame` is a ratchet that only grows. Past estimator.stale (300 ms)
// every observation is rejected, the run can never leave Locking, and the test fails on a machine
// slow or busy enough to overrun a handful of frames -- which is what the Pi 4B does. Re-stamp with
// the clock the estimator actually compares against. Nothing in production has this problem:
// PiCameraSource already stamps with core::Clock::now(). Keeping it out of SyntheticFrameSource
// preserves the fixed-cadence determinism that vision_frame_source_tests pins.
class RealTimeFrames final : public follow::vision::IFrameSource {
public:
  explicit RealTimeFrames(follow::vision::IFrameSource& inner)
    : inner(inner)
  {
  }

  std::optional<follow::vision::Frame> read() override
  {
    std::optional<follow::vision::Frame> frame = this->inner.read();
    if (frame) {
      frame->t = follow::core::Clock::now();
    }
    return frame;
  }

  bool ended() const override { return this->inner.ended(); }

private:
  follow::vision::IFrameSource& inner;
};

// A camera that stops answering, the way a stalled libcamerasrc does: read() blocks inside
// gst_app_sink_pull_sample with no frame, no failure and no end of stream, so the failure budget
// never counts, ended() never fires and the vision thread can never be joined. Everything the
// fail-safe depends on must therefore happen before that join is attempted.
class StallingFrames final : public follow::vision::IFrameSource {
public:
  explicit StallingFrames(follow::vision::IFrameSource& inner)
    : inner(inner)
  {
  }

  ~StallingFrames() override { this->release(); }

  std::optional<follow::vision::Frame> read() override
  {
    std::unique_lock<std::mutex> lock(this->mutex);
    if (this->stalling) {
      this->released.wait(lock, [this] { return this->freed; });
      return std::nullopt;
    }
    lock.unlock();
    return this->inner.read();
  }

  // Only true once the stall has been released, so a stalled source never looks exhausted.
  bool ended() const override
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    return this->freed;
  }

  void stall()
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->stalling = true;
  }

  void release()
  {
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      this->freed = true;
    }
    this->released.notify_all();
  }

private:
  follow::vision::IFrameSource& inner;
  mutable std::mutex mutex;
  std::condition_variable released;
  bool stalling{false};
  bool freed{false};
};

// A camera that dies mid-run the way OpenCV does: cv::Exception derives from std::exception and is
// thrown for, among other things, a frame whose size or type no longer matches what the tracker was
// initialized with -- exactly what a mid-run caps renegotiation on libcamerasrc produces.
class ThrowingFrames final : public follow::vision::IFrameSource {
public:
  ThrowingFrames(follow::vision::IFrameSource& inner, int throwAfter)
    : inner(inner)
    , throwAfter(throwAfter)
  {
  }

  std::optional<follow::vision::Frame> read() override
  {
    if (++this->reads > this->throwAfter) {
      throw std::runtime_error("the camera exploded");
    }
    return this->inner.read();
  }

private:
  follow::vision::IFrameSource& inner;
  int throwAfter;
  int reads{0};
};

// Bearing of a box center through the camera the app was configured with, in the level frame the
// estimator reports. The fake vehicle hovers wings-level, so roll and pitch are zero.
double bearingOfDeg(const follow::core::CameraModel& camera, const follow::core::CameraMount& mount, const cv::Rect& box)
{
  follow::core::Pixel center{box.x + box.width / 2.0, box.y + box.height / 2.0};
  follow::core::Vec3 level = follow::core::bodyToLevel(follow::core::cameraToBody(camera.pixelToRay(center), mount), 0.0, 0.0);
  return follow::core::radToDeg(std::atan2(level.y, level.x));
}

// Every bearing the estimator reported while the target was valid, in order. Read straight out of
// the CSV: readRunLog keeps only the columns sim::checkRun needs, and bearing_deg is not one.
std::vector<double> validBearingsDeg(const std::filesystem::path& logPath)
{
  std::ifstream in(logPath);
  std::string line;
  std::getline(in, line);  // header
  std::vector<double> bearings;
  while (std::getline(in, line)) {
    std::vector<std::string> fields;
    std::string field;
    std::istringstream row(line);
    while (std::getline(row, field, ',')) {
      fields.push_back(field);
    }
    if (fields.size() >= 5 && fields[3] == "1" && !fields[4].empty()) {
      bearings.push_back(std::stod(fields[4]));
    }
  }
  return bearings;
}

// follow.json with the test's overrides, written into `dir`.
std::filesystem::path writeTestConfig(const std::filesystem::path& dir)
{
  // No framebuffer in tests: an empty device disables the overlay.
  nlohmann::json configJson = follow::config::readJsonFile(FOLLOW_CONFIG_DIR "/follow.json");
  configJson["vision"]["framebuffer"] = "";
  configJson["camera"] = FOLLOW_CONFIG_DIR "/camera_imx219_160.json";
  std::filesystem::path path = dir / "follow.json";
  std::ofstream(path) << configJson.dump(2);
  return path;
}

}  // namespace

// Important 5: `nextOverlay = now + overlayPeriod` restarts the period from the draw instant, which
// with the committed defaults (fps 20, overlay_fps 15) quantises the overlay to fps/2 = 10 fps, a
// 33% shortfall against the ~15 fps spec value. Accumulating instead (`nextOverlay += period`) must
// reach the configured rate.
TEST(ShouldDrawOverlay, AccumulatesToTheConfiguredRateInsteadOfHalvingIt)
{
  const auto framePeriod = std::chrono::duration_cast<follow::core::Clock::duration>(std::chrono::duration<double>(1.0 / 20.0));
  const auto overlayPeriod = std::chrono::duration_cast<follow::core::Clock::duration>(std::chrono::duration<double>(1.0 / 15.0));
  follow::core::TimePoint now = follow::core::Clock::now();
  follow::core::TimePoint nextOverlay = now;
  int draws = 0;
  const int ticks = 100;  // 5 s of frames at 20 fps
  for (int i = 0; i < ticks; ++i) {
    if (follow::vision::detail::shouldDrawOverlay(now, nextOverlay, overlayPeriod)) {
      ++draws;
    }
    now += framePeriod;
  }
  // 5 s at 15 fps is 75 draws; the bug this fixes produces exactly 50 (10 fps). Allow +-1 for the
  // boundary tick.
  EXPECT_GE(draws, 74);
  EXPECT_LE(draws, 76);
}

TEST(ShouldDrawOverlay, DoesNotDrawBeforeItsScheduledInstant)
{
  auto period = std::chrono::milliseconds{60};
  follow::core::TimePoint nextOverlay = follow::core::Clock::now();
  EXPECT_FALSE(follow::vision::detail::shouldDrawOverlay(nextOverlay - std::chrono::milliseconds{1}, nextOverlay, period));
}

// After a long stall (a blocked framebuffer write, a slow machine), the schedule must catch up to
// `now` rather than bursting through every slot it missed while behind.
TEST(ShouldDrawOverlay, RecoversAfterAStallWithoutBurstingThroughMissedSlots)
{
  const auto period = std::chrono::milliseconds{60};
  const auto tick = std::chrono::milliseconds{10};
  follow::core::TimePoint now = follow::core::Clock::now();
  follow::core::TimePoint nextOverlay = now;
  ASSERT_TRUE(follow::vision::detail::shouldDrawOverlay(now, nextOverlay, period));
  now += std::chrono::milliseconds{500};                                             // long stall: nextOverlay is now far behind `now`
  ASSERT_TRUE(follow::vision::detail::shouldDrawOverlay(now, nextOverlay, period));  // catches up, clamps to `now`

  int draws = 0;
  for (int i = 0; i < 12; ++i) {
    now += tick;
    if (follow::vision::detail::shouldDrawOverlay(now, nextOverlay, period)) {
      ++draws;
    }
  }
  EXPECT_LE(draws, 3);
}

// Runs the whole follow_app --hw wiring against a fake autopilot with synthetic camera frames for
// about 5 s: the pilot engages after 1 s, the tracker locks on the centered target and follows.
TEST(HwAppTest, EngagesAndFollowsASyntheticTarget)
{
  std::filesystem::path dir = std::filesystem::temp_directory_path() / ("follow_hw_app_test_" + std::to_string(::getpid()));
  std::filesystem::create_directories(dir);
  TempDirGuard guard(dir);
  std::filesystem::path configPath = writeTestConfig(dir);

  // The pilot engages shortly after the app connects. It has to be soon: the synthetic target crosses
  // the frame at 3 px per frame, so a second of dead time before LockCenter would seed the tracker on
  // the background the target has already left -- which is what this test used to do.
  follow::test::FakeAutopilot fc(0.3);
  fc.start();
  follow::vision::SyntheticFrameSource frames(640, 480, follow::core::Clock::now());
  RealTimeFrames realTime(frames);
  follow::vision::HwAppOptions options{
    .configPath = configPath, .link = "udp:0:127.0.0.1:" + std::to_string(fc.port()), .logPath = dir / "run.csv"};
  std::atomic<bool> stop{false};
  Stopper stopper(stop, std::chrono::seconds{4});
  std::ostringstream out;

  follow::vision::runHwApp(options, realTime, stop, out);

  // The fail-safe: ArduPilot holds the last velocity target until its guided timeout (3 s, the rule
  // FakeAutopilot encodes), so an app that just stops leaves the vehicle coasting at its following
  // speed. runHwApp must command zero before it returns. The datagram may still be in the socket
  // when runHwApp returns, so wait for the FC's receive loop to pick it up.
  std::optional<follow::core::VelocityCmd> last;
  for (int i = 0; i < 100 && !(last && last->vx == 0.0 && last->yawRate == 0.0); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds{5});
    last = fc.lastSetpoint();
  }
  fc.stop();
  ASSERT_TRUE(last.has_value()) << out.str();
  EXPECT_DOUBLE_EQ(last->vx, 0.0) << out.str();
  EXPECT_DOUBLE_EQ(last->yawRate, 0.0) << out.str();

  std::ifstream log(options.logPath);
  std::vector<follow::sim::StepRecord> steps = follow::runtime::readRunLog(log);
  auto hasState = [&steps](follow::core::State state) {
    return std::any_of(steps.begin(), steps.end(), [state](const follow::sim::StepRecord& step) { return step.state == state; });
  };
  EXPECT_FALSE(steps.empty()) << out.str();
  EXPECT_TRUE(hasState(follow::core::State::Locking)) << out.str();
  EXPECT_TRUE(hasState(follow::core::State::Following)) << out.str();
  EXPECT_NE(out.str().find("follow_app --hw: tracker kcf"), std::string::npos) << out.str();

  // ... and that it really follows, which reaching Following does not by itself prove: Following is
  // entered as soon as the tracker returns any box the estimator accepts, and a box seeded on the
  // static background is accepted just as readily -- it is stable, so nothing downstream objects.
  // The target crosses the frame at 3 px per frame, so a tracker that is on it sweeps the bearing by
  // tens of degrees while one on the background holds a constant bearing.
  follow::config::AppConfig app = follow::config::loadAppConfig(configPath);
  std::unique_ptr<follow::core::CameraModel> camera = follow::config::makeCameraModel(app.camera);
  std::vector<double> bearings = validBearingsDeg(options.logPath);
  double truth = bearingOfDeg(*camera, app.camera.mount, frames.groundTruth());
  ASSERT_FALSE(bearings.empty()) << out.str();
  // The threshold is set by what separates the two hypotheses, not by what a fast machine reaches.
  // The run is bounded by wall clock, so a loaded machine completes fewer frames and the target --
  // which advances 3 px per frame, not per millisecond -- sweeps proportionally less: measured near
  // 60 deg on an idle Mac but as little as 18.8 deg on a Pi 4B under four-way CPU load. A tracker
  // latched onto the static background reads 0.46 deg. 10 deg sits far outside the noise of the
  // first and nowhere near the second, so it discriminates on every machine instead of encoding
  // this host's speed.
  EXPECT_GT(bearings.back() - bearings.front(), 10.0) << out.str();
  // The estimator reports the bearing relative to the *current* heading and the vehicle is yawing at
  // its 45 deg/s limit to chase, so the reported bearing trails the bearing of the frame it came
  // from by the yaw accumulated since capture -- measured at ~14 deg here. The tolerance covers that
  // and a slower machine's larger lag; what it does not cover is a tracker that is not on the target
  // at all, which read a constant -18 deg against a ground truth of +60 deg.
  EXPECT_NEAR(bearings.back(), truth, 30.0) << out.str();
}

// A bad config must produce one red test, not an aborted binary: everything runHwApp throws unwinds
// through the stopper thread on its way out.
TEST(HwAppTest, ReportsAnUnreadableConfigInsteadOfAborting)
{
  std::filesystem::path dir = std::filesystem::temp_directory_path() / ("follow_hw_app_config_test_" + std::to_string(::getpid()));
  std::filesystem::create_directories(dir);
  TempDirGuard guard(dir);

  follow::vision::SyntheticFrameSource frames(640, 480, follow::core::Clock::now());
  follow::vision::HwAppOptions options{.configPath = dir / "does_not_exist.json", .link = "udp:0", .logPath = dir / "run.csv"};
  std::atomic<bool> stop{false};
  Stopper stopper(stop, std::chrono::seconds{5});
  std::ostringstream out;

  EXPECT_THROW(follow::vision::runHwApp(options, frames, stop, out), follow::config::ConfigError);
}

// An exception escaping a worker thread calls std::terminate(): the process dies instantly, no
// destructor runs, the run log is truncated and -- worst of all -- no zero setpoint is sent. The
// vision thread newly makes this reachable because it calls into OpenCV on every frame, and OpenCV
// signals every error by throwing.
TEST(HwAppTest, ReportsAFailedWorkerThreadAfterCommandingZero)
{
  std::filesystem::path dir = std::filesystem::temp_directory_path() / ("follow_hw_app_throw_test_" + std::to_string(::getpid()));
  std::filesystem::create_directories(dir);
  TempDirGuard guard(dir);
  std::filesystem::path configPath = writeTestConfig(dir);

  follow::test::FakeAutopilot fc(0.0);
  fc.start();
  follow::vision::SyntheticFrameSource frames(640, 480, follow::core::Clock::now());
  ThrowingFrames throwing(frames, 20);  // 1 s of frames, long enough to have commanded something
  follow::vision::HwAppOptions options{
    .configPath = configPath, .link = "udp:0:127.0.0.1:" + std::to_string(fc.port()), .logPath = dir / "run.csv"};
  std::atomic<bool> stop{false};
  Stopper stopper(stop, std::chrono::seconds{10});  // must never fire: the throw ends the run
  std::ostringstream out;

  try {
    follow::vision::runHwApp(options, throwing, stop, out);
    ADD_FAILURE() << "runHwApp returned normally: " << out.str();
  }
  catch (const std::runtime_error& e) {
    EXPECT_NE(std::string(e.what()).find("the camera exploded"), std::string::npos);
  }

  std::optional<follow::core::VelocityCmd> last;
  for (int i = 0; i < 100 && !(last && last->vx == 0.0 && last->yawRate == 0.0); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds{5});
    last = fc.lastSetpoint();
  }
  fc.stop();
  // The fail-safe goes out even on the failure path, and before runHwApp rethrows.
  ASSERT_TRUE(last.has_value()) << out.str();
  EXPECT_DOUBLE_EQ(last->vx, 0.0) << out.str();
  EXPECT_DOUBLE_EQ(last->yawRate, 0.0) << out.str();
}

// The fail-safe must not depend on the camera thread being joinable. A stalled GStreamer pipeline
// blocks the vision thread for ever, and the zero setpoint has to be on the wire before anything
// waits on it -- otherwise this degrades to exactly the behaviour the fail-safe exists to remove:
// the FC holding the last following setpoint for its whole guided timeout.
TEST(HwAppTest, SendsTheFailSafeEvenWhenTheCameraThreadIsStuck)
{
  std::filesystem::path dir = std::filesystem::temp_directory_path() / ("follow_hw_app_stall_test_" + std::to_string(::getpid()));
  std::filesystem::create_directories(dir);
  TempDirGuard guard(dir);
  std::filesystem::path configPath = writeTestConfig(dir);

  follow::test::FakeAutopilot fc(0.3);
  fc.start();
  follow::vision::SyntheticFrameSource frames(640, 480, follow::core::Clock::now());
  RealTimeFrames realTime(frames);
  StallingFrames stalling(realTime);
  follow::vision::HwAppOptions options{
    .configPath = configPath, .link = "udp:0:127.0.0.1:" + std::to_string(fc.port()), .logPath = dir / "run.csv"};
  std::atomic<bool> stop{false};
  std::ostringstream out;
  std::thread app([&] { follow::vision::runHwApp(options, stalling, stop, out); });

  auto moving = [&fc] {
    std::optional<follow::core::VelocityCmd> last = fc.lastSetpoint();
    return last && (last->vx != 0.0 || last->yawRate != 0.0);
  };
  for (int i = 0; i < 600 && !moving(); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds{5});
  }
  bool wasMoving = moving();
  // Stall the camera and stop the app in the same breath, so the estimator's 300 ms staleness rule
  // cannot be what commands zero: this has to be runHwApp's own fail-safe.
  stalling.stall();
  stop = true;

  std::optional<follow::core::VelocityCmd> last;
  for (int i = 0; i < 400 && !(last && last->vx == 0.0 && last->yawRate == 0.0); ++i) {
    std::this_thread::sleep_for(std::chrono::milliseconds{5});
    last = fc.lastSetpoint();
  }
  stalling.release();
  app.join();
  fc.stop();

  EXPECT_TRUE(wasMoving) << "the vehicle was never commanded to move, so zero proves nothing\n" << out.str();
  ASSERT_TRUE(last.has_value()) << out.str();
  EXPECT_DOUBLE_EQ(last->vx, 0.0) << out.str();
  EXPECT_DOUBLE_EQ(last->yawRate, 0.0) << out.str();
}
