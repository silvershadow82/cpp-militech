#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <filesystem>
#include <fstream>
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
#include "follow/core/Types.h"
#include "follow/runtime/HwApp.h"
#include "follow/runtime/RunLog.h"
#include "follow/vision/FrameSource.h"

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

// Runs the whole follow_app --hw wiring against a fake autopilot with synthetic camera frames for
// about 5 s: the pilot engages after 1 s, the tracker locks on the centered target and follows.
TEST(HwAppTest, EngagesAndFollowsASyntheticTarget)
{
  std::filesystem::path dir = std::filesystem::temp_directory_path() / ("follow_hw_app_test_" + std::to_string(::getpid()));
  std::filesystem::create_directories(dir);
  TempDirGuard guard(dir);
  std::filesystem::path configPath = writeTestConfig(dir);

  follow::test::FakeAutopilot fc(1.0);
  fc.start();
  follow::vision::SyntheticFrameSource frames(640, 480, follow::core::Clock::now());
  follow::runtime::HwAppOptions options{
    .configPath = configPath, .link = "udp:0:127.0.0.1:" + std::to_string(fc.port()), .logPath = dir / "run.csv"};
  std::atomic<bool> stop{false};
  Stopper stopper(stop, std::chrono::seconds{5});
  std::ostringstream out;

  follow::runtime::runHwApp(options, frames, stop, out);

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
}

// A bad config must produce one red test, not an aborted binary: everything runHwApp throws unwinds
// through the stopper thread on its way out.
TEST(HwAppTest, ReportsAnUnreadableConfigInsteadOfAborting)
{
  std::filesystem::path dir = std::filesystem::temp_directory_path() / ("follow_hw_app_config_test_" + std::to_string(::getpid()));
  std::filesystem::create_directories(dir);
  TempDirGuard guard(dir);

  follow::vision::SyntheticFrameSource frames(640, 480, follow::core::Clock::now());
  follow::runtime::HwAppOptions options{.configPath = dir / "does_not_exist.json", .link = "udp:0", .logPath = dir / "run.csv"};
  std::atomic<bool> stop{false};
  Stopper stopper(stop, std::chrono::seconds{5});
  std::ostringstream out;

  EXPECT_THROW(follow::runtime::runHwApp(options, frames, stop, out), follow::config::ConfigError);
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
  follow::runtime::HwAppOptions options{
    .configPath = configPath, .link = "udp:0:127.0.0.1:" + std::to_string(fc.port()), .logPath = dir / "run.csv"};
  std::atomic<bool> stop{false};
  Stopper stopper(stop, std::chrono::seconds{10});  // must never fire: the throw ends the run
  std::ostringstream out;

  try {
    follow::runtime::runHwApp(options, throwing, stop, out);
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
