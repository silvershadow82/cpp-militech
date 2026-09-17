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
