#include <gtest/gtest.h>

#include "TestTime.h"
#include "follow/core/Supervisor.h"

using namespace follow::core;
using follow::test::at;

namespace {

// Feeds the supervisor with a fresh heartbeat unless told otherwise.
class SupervisorDriver {
public:
  State tick(double tSec, uint32_t mode, bool targetValid)
  {
    this->heartbeat = at(tSec);
    return this->supervisor.update(at(tSec), this->heartbeat, mode, targetValid);
  }

  State tickWithoutHeartbeat(double tSec, uint32_t mode, bool targetValid)
  {
    return this->supervisor.update(at(tSec), this->heartbeat, mode, targetValid);
  }

  // Idle in LOITER at t = 0, GUIDED at t = 0.05.
  void engage()
  {
    this->tick(0.0, kModeLoiter, false);
    this->tick(0.05, kModeGuided, false);
  }

private:
  Supervisor supervisor{SupervisorConfig{}};
  std::optional<TimePoint> heartbeat{};
};

}  // namespace

TEST(Supervisor, StartsIdle)
{
  Supervisor supervisor{SupervisorConfig{}};
  EXPECT_EQ(supervisor.state(), State::Idle);
}

TEST(Supervisor, EngagesWhenModeChangesToGuided)
{
  SupervisorDriver driver;
  EXPECT_EQ(driver.tick(0.0, kModeLoiter, false), State::Idle);
  EXPECT_EQ(driver.tick(0.05, kModeGuided, false), State::Locking);
}

TEST(Supervisor, DoesNotEngageWhenAlreadyGuidedAtStart)
{
  SupervisorDriver driver;
  EXPECT_EQ(driver.tick(0.0, kModeGuided, true), State::Idle);
  EXPECT_EQ(driver.tick(0.05, kModeGuided, true), State::Idle);
}

TEST(Supervisor, LockingBecomesFollowingOnValidTarget)
{
  SupervisorDriver driver;
  driver.engage();
  EXPECT_EQ(driver.tick(0.1, kModeGuided, true), State::Following);
}

TEST(Supervisor, LockingTimesOutToLost)
{
  SupervisorDriver driver;
  driver.engage();
  EXPECT_EQ(driver.tick(1.0, kModeGuided, false), State::Locking);
  EXPECT_EQ(driver.tick(1.05, kModeGuided, false), State::Lost);
}

TEST(Supervisor, FollowingGoesLostAndRecovers)
{
  SupervisorDriver driver;
  driver.engage();
  driver.tick(0.1, kModeGuided, true);
  EXPECT_EQ(driver.tick(0.15, kModeGuided, false), State::Lost);
  EXPECT_EQ(driver.tick(0.2, kModeGuided, true), State::Following);
}

TEST(Supervisor, LostTimesOutToHold)
{
  SupervisorDriver driver;
  driver.engage();
  driver.tick(0.1, kModeGuided, true);
  driver.tick(0.15, kModeGuided, false);
  EXPECT_EQ(driver.tick(3.1, kModeGuided, false), State::Lost);
  EXPECT_EQ(driver.tick(3.15, kModeGuided, false), State::Hold);
}

TEST(Supervisor, HoldIgnoresTargetUntilModeLeavesGuided)
{
  SupervisorDriver driver;
  driver.engage();
  driver.tick(1.05, kModeGuided, false);
  driver.tick(4.05, kModeGuided, false);
  ASSERT_EQ(driver.tick(4.1, kModeGuided, false), State::Hold);

  EXPECT_EQ(driver.tick(4.15, kModeGuided, true), State::Hold);
  EXPECT_EQ(driver.tick(4.2, kModeLoiter, true), State::Idle);
}

TEST(Supervisor, LeavingGuidedFromEachEngagedStateGoesIdle)
{
  SupervisorDriver locking;
  locking.engage();
  EXPECT_EQ(locking.tick(0.1, kModeLoiter, false), State::Idle);

  SupervisorDriver following;
  following.engage();
  following.tick(0.1, kModeGuided, true);
  EXPECT_EQ(following.tick(0.15, kModeAltHold, true), State::Idle);

  SupervisorDriver lost;
  lost.engage();
  lost.tick(1.05, kModeGuided, false);
  EXPECT_EQ(lost.tick(1.1, kModeLoiter, false), State::Idle);
}

TEST(Supervisor, HeartbeatTimeoutGivesNoFc)
{
  SupervisorDriver driver;
  driver.engage();
  driver.tick(0.1, kModeGuided, true);
  EXPECT_EQ(driver.tickWithoutHeartbeat(2.1, kModeGuided, true), State::Following);
  EXPECT_EQ(driver.tickWithoutHeartbeat(2.15, kModeGuided, true), State::NoFc);
}

TEST(Supervisor, RecoveryFromNoFcInGuidedStaysIdleUntilSwitchCycled)
{
  // Setup: following, then the FC link drops
  SupervisorDriver driver;
  driver.engage();
  driver.tick(0.1, kModeGuided, true);
  ASSERT_EQ(driver.tickWithoutHeartbeat(2.5, kModeGuided, true), State::NoFc);

  // Run + Assert: link returns with the FC still in GUIDED
  EXPECT_EQ(driver.tick(2.55, kModeGuided, true), State::Idle);
  EXPECT_EQ(driver.tick(2.6, kModeGuided, true), State::Idle);
  EXPECT_EQ(driver.tick(2.65, kModeLoiter, true), State::Idle);
  EXPECT_EQ(driver.tick(2.7, kModeGuided, true), State::Locking);
}
