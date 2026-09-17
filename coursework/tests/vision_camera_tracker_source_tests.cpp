#include <gtest/gtest.h>

#include <memory>

#include "follow/runtime/Channels.h"
#include "follow/vision/CameraTrackerSource.h"
#include "follow/vision/FrameSource.h"
#include "follow/vision/Tracker.h"

namespace {

using follow::core::BBox;
using follow::core::TrackerRequest;
using follow::core::TrackerRequestKind;

const BBox kLockBox{.x = 272.0, .y = 192.0, .w = 96.0, .h = 96.0};  // Core::lockBox() for 640x480, lock_box_frac 0.20

struct Fixture {
  follow::runtime::Channels channels;
  follow::vision::SyntheticFrameSource frames{640, 480, follow::core::Clock::now()};
  follow::vision::CameraTrackerSource source{frames, follow::vision::makeTracker("kcf"), follow::vision::CameraTrackerConfig{}, channels};
};

// A source the test drives: `miss` makes one read() fail the way a live camera drops a buffer,
// `exhausted` reports the end of the stream the way a clip does at EOF.
class ScriptedFrames final : public follow::vision::IFrameSource {
public:
  explicit ScriptedFrames(follow::vision::IFrameSource& inner)
    : inner(inner)
  {
  }

  std::optional<follow::vision::Frame> read() override
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
  follow::vision::IFrameSource& inner;
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
  cv::Rect grown = follow::vision::expandBox(cv::Rect(100, 100, 40, 80), 1.5, cv::Size(640, 480));
  EXPECT_EQ(grown, cv::Rect(90, 80, 60, 120));

  cv::Rect clipped = follow::vision::expandBox(cv::Rect(0, 0, 40, 40), 1.5, cv::Size(640, 480));
  EXPECT_EQ(clipped.x, 0);
  EXPECT_EQ(clipped.y, 0);
  EXPECT_LE(clipped.br().x, 640);
}

TEST(BoxConversion, RoundTripsIntegerBoxes)
{
  cv::Rect r(12, 34, 56, 78);
  EXPECT_EQ(follow::vision::toRect(follow::vision::toBBox(r)), r);
}

TEST(CameraTrackerSource, AMissedFrameKeepsIteratingAndPublishesNothing)
{
  // A dropped buffer on a live camera is transient, not the end of the stream. Ending the app on
  // one would leave the FC holding the last velocity setpoint for its guided timeout; publishing
  // nothing instead lets the estimator's staleness rule run, so the core goes Lost and commands
  // zero all by itself.
  follow::runtime::Channels channels;
  follow::vision::SyntheticFrameSource frames{640, 480, follow::core::Clock::now()};
  ScriptedFrames scripted{frames};
  follow::vision::CameraTrackerSource source{scripted, follow::vision::makeTracker("kcf"), follow::vision::CameraTrackerConfig{}, channels};
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
  follow::runtime::Channels channels;
  follow::vision::SyntheticFrameSource frames{640, 480, follow::core::Clock::now()};
  ScriptedFrames scripted{frames};
  follow::vision::CameraTrackerSource source{scripted, follow::vision::makeTracker("kcf"), follow::vision::CameraTrackerConfig{}, channels};
  EXPECT_TRUE(source.iterate());

  scripted.exhausted = true;
  EXPECT_FALSE(source.iterate());
}
