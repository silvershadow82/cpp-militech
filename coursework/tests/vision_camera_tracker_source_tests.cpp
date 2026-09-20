#include <gtest/gtest.h>

#include <memory>

#include "providers/CameraTrackerSource.h"
#include "interfaces/IFrameSource.h"
#include "providers/FrameSource.h"
#include "util/Channels.h"
#include "vision/TrackerFactory.h"

namespace {

using follow::control::TrackerRequest;
using follow::control::TrackerRequestKind;
using follow::models::BBox;

const BBox kLockBox{.x = 272.0, .y = 192.0, .w = 96.0, .h = 96.0};  // Core::lockBox() for 640x480, lock_box_frac 0.20

struct Fixture {
  follow::util::Channels channels;
  follow::providers::SyntheticFrameSource frames{640, 480, follow::models::Clock::now()};
  follow::providers::CameraTrackerSource source{
    frames, follow::vision::makeTracker("kcf"), follow::providers::CameraTrackerConfig{}, channels};
};

// A source the test drives: `miss` makes one read() fail the way a live camera drops a buffer,
// `exhausted` reports the end of the stream the way a clip does at EOF.
class ScriptedFrames final : public follow::interfaces::IFrameSource {
public:
  explicit ScriptedFrames(follow::interfaces::IFrameSource& inner)
    : inner(inner)
  {
  }

  std::optional<follow::interfaces::Frame> read() override
  {
    if (this->miss) {
      this->miss = false;
      return std::nullopt;
    }
    if (this->exhausted) {
      return std::nullopt;
    }
    return this->inner.read();
  }

  bool ended() const override { return this->exhausted; }

  bool miss{false};
  bool exhausted{false};

private:
  follow::interfaces::IFrameSource& inner;
};

// Counts init() calls and reports whatever the test tells it to. A real KCF hides both: it never
// says how often it was re-seeded, and it succeeds on almost any patch it is given.
class CountingTracker final : public follow::interfaces::ITracker {
public:
  void init(const cv::Mat&, const cv::Rect& box) override
  {
    ++this->inits;
    this->lastInit = box;
  }

  std::optional<cv::Rect> update(const cv::Mat&) override { return this->result; }

  int inits{0};
  cv::Rect lastInit{};
  std::optional<cv::Rect> result{cv::Rect(300, 220, 40, 40)};  // succeeding until a test clears it
};

// A fixture whose tracker the test controls; `tracker` stays valid for as long as `source` does.
struct CountingFixture {
  CountingFixture()
    : tracker(new CountingTracker)
    , source(frames, std::unique_ptr<follow::interfaces::ITracker>(tracker), follow::providers::CameraTrackerConfig{}, channels)
  {
  }

  follow::util::Channels channels;
  follow::providers::SyntheticFrameSource frames{640, 480, follow::models::Clock::now()};
  CountingTracker* tracker;
  follow::providers::CameraTrackerSource source;
};

}  // namespace

TEST(CameraTrackerSource, PublishesNothingBeforeLockCenter)
{
  Fixture f;
  EXPECT_TRUE(f.source.iterate());
  EXPECT_FALSE(f.source.locked());
  EXPECT_FALSE(f.channels.observation.read().has_value());
  EXPECT_TRUE(f.source.lastFrame().has_value());  // frames keep flowing for the overlay
}

TEST(CameraTrackerSource, LockCenterPublishesAnObservationEveryFrame)
{
  Fixture f;
  f.channels.trackerRequests.push(TrackerRequest{.kind = TrackerRequestKind::LockCenter, .hint = kLockBox});
  f.source.iterate();
  ASSERT_TRUE(f.source.locked());

  for (int i = 0; i < 30; ++i) {
    f.source.iterate();
  }
  auto slot = f.channels.observation.read();
  ASSERT_TRUE(slot.has_value());
  EXPECT_EQ(slot->sequence, 31u);  // the LockCenter pass publishes too
  EXPECT_TRUE(slot->value.ok);
  EXPECT_DOUBLE_EQ(slot->value.confidence, 1.0);
  EXPECT_EQ(slot->value.tFrame, f.source.lastFrame()->t);
  EXPECT_EQ(slot->t, slot->value.tFrame);
}

TEST(CameraTrackerSource, ReacquireWhileLockedKeepsTracking)
{
  Fixture f;
  f.channels.trackerRequests.push(TrackerRequest{.kind = TrackerRequestKind::LockCenter, .hint = kLockBox});
  f.source.iterate();
  f.channels.trackerRequests.push(TrackerRequest{.kind = TrackerRequestKind::Reacquire, .hint = kLockBox});
  f.source.iterate();
  EXPECT_TRUE(f.source.locked());
  EXPECT_TRUE(f.channels.observation.read()->value.ok);
}

TEST(CameraTrackerSource, UnlockStopsPublication)
{
  Fixture f;
  f.channels.trackerRequests.push(TrackerRequest{.kind = TrackerRequestKind::LockCenter, .hint = kLockBox});
  f.source.iterate();
  f.channels.trackerRequests.push(TrackerRequest{.kind = TrackerRequestKind::Unlock, .hint = {}});
  f.source.iterate();
  uint64_t before = f.channels.observation.read()->sequence;
  f.source.iterate();
  f.source.iterate();
  EXPECT_FALSE(f.source.locked());
  EXPECT_EQ(f.channels.observation.read()->sequence, before);
}

TEST(CameraTrackerSource, ReacquireBeforeAnyLockIsIgnored)
{
  Fixture f;
  f.channels.trackerRequests.push(TrackerRequest{.kind = TrackerRequestKind::Reacquire, .hint = kLockBox});
  f.source.iterate();
  EXPECT_FALSE(f.source.locked());
  EXPECT_FALSE(f.channels.observation.read().has_value());
}

TEST(ExpandBox, GrowsAroundTheCenterAndClipsToTheImage)
{
  cv::Rect grown = follow::providers::expandBox(cv::Rect(100, 100, 40, 80), 1.5, cv::Size(640, 480));
  EXPECT_EQ(grown, cv::Rect(90, 80, 60, 120));

  cv::Rect clipped = follow::providers::expandBox(cv::Rect(0, 0, 40, 40), 1.5, cv::Size(640, 480));
  EXPECT_EQ(clipped.x, 0);
  EXPECT_EQ(clipped.y, 0);
  EXPECT_LE(clipped.br().x, 640);
}

TEST(BoxConversion, RoundTripsIntegerBoxes)
{
  cv::Rect r(12, 34, 56, 78);
  EXPECT_EQ(follow::providers::toRect(follow::providers::toBBox(r)), r);
}

TEST(CameraTrackerSource, AMissedFrameKeepsIteratingAndPublishesNothing)
{
  // A dropped buffer on a live camera is transient, not the end of the stream. Ending the app on
  // one would leave the FC holding the last velocity setpoint for its guided timeout; publishing
  // nothing instead lets the estimator's staleness rule run, so the core goes Lost and commands
  // zero all by itself.
  follow::util::Channels channels;
  follow::providers::SyntheticFrameSource frames{640, 480, follow::models::Clock::now()};
  ScriptedFrames scripted{frames};
  follow::providers::CameraTrackerSource source{
    scripted, follow::vision::makeTracker("kcf"), follow::providers::CameraTrackerConfig{}, channels};
  channels.trackerRequests.push(TrackerRequest{.kind = TrackerRequestKind::LockCenter, .hint = kLockBox});
  source.iterate();
  uint64_t before = channels.observation.read()->sequence;

  scripted.miss = true;
  EXPECT_TRUE(source.iterate());
  EXPECT_EQ(channels.observation.read()->sequence, before);  // nothing published for the missing frame
  EXPECT_EQ(source.missedFrames(), 1u);
  EXPECT_TRUE(source.lastFrame().has_value());  // the overlay keeps the last good frame

  EXPECT_TRUE(source.iterate());  // and the next real frame carries on as before
  EXPECT_GT(channels.observation.read()->sequence, before);
  EXPECT_EQ(source.missedFrames(), 1u);
}

TEST(CameraTrackerSource, StopsOnlyWhenTheSourceReportsItIsExhausted)
{
  follow::util::Channels channels;
  follow::providers::SyntheticFrameSource frames{640, 480, follow::models::Clock::now()};
  ScriptedFrames scripted{frames};
  follow::providers::CameraTrackerSource source{
    scripted, follow::vision::makeTracker("kcf"), follow::providers::CameraTrackerConfig{}, channels};
  EXPECT_TRUE(source.iterate());

  scripted.exhausted = true;
  EXPECT_FALSE(source.iterate());
  // The terminal read is an ending, not a miss: it must not inflate missedFrames() (iterate() checks
  // frames.ended() before counting a nullopt read as a miss).
  EXPECT_EQ(source.missedFrames(), 0u);
}

TEST(CameraTrackerSource, ReacquireIsANoOpWhileTheTrackerStillSucceeds)
{
  // The core enters Lost for estimator-side reasons -- staleness, border margin, area jump -- with
  // the tracker still locked on the target, and emits one Reacquire. Re-seeding then throws away a
  // good lock for a box that is 1.5x too big and mostly background, which KCF will happily follow
  // and report with confidence 1.0 for the rest of the flight. The spec calls for a no-op.
  CountingFixture f;
  f.channels.trackerRequests.push(TrackerRequest{.kind = TrackerRequestKind::LockCenter, .hint = kLockBox});
  f.source.iterate();
  ASSERT_EQ(f.tracker->inits, 1);
  cv::Rect published = follow::providers::toRect(f.channels.observation.read()->value.box);

  f.channels.trackerRequests.push(TrackerRequest{.kind = TrackerRequestKind::Reacquire, .hint = kLockBox});
  f.source.iterate();

  EXPECT_EQ(f.tracker->inits, 1);  // not re-seeded at all
  EXPECT_TRUE(f.source.locked());
  EXPECT_EQ(follow::providers::toRect(f.channels.observation.read()->value.box), published);
}

TEST(CameraTrackerSource, ReacquireReseedsAtMostOncePerPeriod)
{
  // Once the tracker really has failed, re-seeding is right -- but at most once per reacquirePeriod,
  // so a burst of requests cannot restart KCF on every frame.
  CountingFixture f;
  f.channels.trackerRequests.push(TrackerRequest{.kind = TrackerRequestKind::LockCenter, .hint = kLockBox});
  f.source.iterate();
  f.tracker->result.reset();
  f.source.iterate();  // the update fails, so the next Reacquire is honoured
  ASSERT_EQ(f.tracker->inits, 1);

  f.channels.trackerRequests.push(TrackerRequest{.kind = TrackerRequestKind::Reacquire, .hint = kLockBox});
  f.source.iterate();
  f.channels.trackerRequests.push(TrackerRequest{.kind = TrackerRequestKind::Reacquire, .hint = kLockBox});
  f.source.iterate();  // 50 ms later, well inside the 500 ms period

  EXPECT_EQ(f.tracker->inits, 2);  // the LockCenter plus exactly one re-seed
  EXPECT_EQ(f.tracker->lastInit, follow::providers::expandBox(follow::providers::toRect(kLockBox), 1.5, cv::Size(640, 480)));
}

TEST(CameraTrackerSource, ReacquireIsRetriedEveryPeriodWhileTheTrackerKeepsFailing)
{
  // Core::step emits a TrackerRequest only on a state edge, so exactly one Reacquire is produced per
  // entry into Lost. The retry is therefore the adapter's job: without it reacquisition is a single
  // attempt at the instant the tracker first failed and reacquire_period_ms is dead config.
  //
  // Read what this pins precisely: the cadence for a tracker that keeps failing after re-init, which
  // is what CountingTracker does. KCF and CSRT do not -- they return a box for almost any in-image
  // init, including one seeded on background -- so the latch clears on the re-seed's own update and
  // production performs a single re-seed per Lost entry. Retrying on the same hint would only
  // re-seed the same patch, so that is the intended behaviour, not a gap; distinguishing a good
  // re-seed from a background latch needs a confidence signal neither tracker exposes.
  CountingFixture f;
  f.channels.trackerRequests.push(TrackerRequest{.kind = TrackerRequestKind::LockCenter, .hint = kLockBox});
  f.source.iterate();
  f.tracker->result.reset();
  f.source.iterate();  // the update the core sees fail, which is what makes it emit Reacquire
  ASSERT_EQ(f.tracker->inits, 1);

  f.channels.trackerRequests.push(TrackerRequest{.kind = TrackerRequestKind::Reacquire, .hint = kLockBox});
  for (int i = 0; i < 60; ++i) {  // 3 s of frames 50 ms apart, the whole lost_timeout_ms window
    f.source.iterate();
  }

  EXPECT_EQ(f.tracker->inits, 1 + 6);  // the LockCenter plus one re-seed every 500 ms
}

TEST(CameraTrackerSource, RetryingStopsWhenTheTrackerRecovers)
{
  CountingFixture f;
  f.channels.trackerRequests.push(TrackerRequest{.kind = TrackerRequestKind::LockCenter, .hint = kLockBox});
  f.source.iterate();
  f.tracker->result.reset();
  f.source.iterate();
  f.channels.trackerRequests.push(TrackerRequest{.kind = TrackerRequestKind::Reacquire, .hint = kLockBox});
  f.source.iterate();
  ASSERT_EQ(f.tracker->inits, 2);

  f.tracker->result = cv::Rect(300, 220, 40, 40);
  for (int i = 0; i < 40; ++i) {  // 2 s: four retries' worth, had the latch not been cleared
    f.source.iterate();
  }

  EXPECT_EQ(f.tracker->inits, 2);
  EXPECT_TRUE(f.channels.observation.read()->value.ok);
}

TEST(CameraTrackerSource, UnlockCancelsAnOutstandingReacquire)
{
  CountingFixture f;
  f.channels.trackerRequests.push(TrackerRequest{.kind = TrackerRequestKind::LockCenter, .hint = kLockBox});
  f.source.iterate();
  f.tracker->result.reset();
  f.source.iterate();
  f.channels.trackerRequests.push(TrackerRequest{.kind = TrackerRequestKind::Reacquire, .hint = kLockBox});
  f.source.iterate();
  ASSERT_EQ(f.tracker->inits, 2);

  f.channels.trackerRequests.push(TrackerRequest{.kind = TrackerRequestKind::Unlock, .hint = {}});
  for (int i = 0; i < 40; ++i) {
    f.source.iterate();
  }

  EXPECT_FALSE(f.source.locked());
  EXPECT_EQ(f.tracker->inits, 2);
}
