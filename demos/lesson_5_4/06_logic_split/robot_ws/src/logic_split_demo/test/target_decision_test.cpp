#include <gtest/gtest.h>

#include "logic_split_demo/target_decision.hpp"

namespace {

constexpr logic_split_demo::DecisionConfig kConfig{0.8F, 30.0F};

}  // namespace

TEST(TargetDecision, HiddenTargetIsIdle)
{
  const auto decision = logic_split_demo::decide(logic_split_demo::TargetInput{false, 0.95F, 10.0F}, kConfig);

  EXPECT_EQ(logic_split_demo::Decision::kIdle, decision);
}

TEST(TargetDecision, LowConfidenceIsIdle)
{
  const auto decision = logic_split_demo::decide(logic_split_demo::TargetInput{true, 0.40F, 10.0F}, kConfig);

  EXPECT_EQ(logic_split_demo::Decision::kIdle, decision);
}

TEST(TargetDecision, FarTargetIsTrackOnly)
{
  const auto decision = logic_split_demo::decide(logic_split_demo::TargetInput{true, 0.90F, 45.0F}, kConfig);

  EXPECT_EQ(logic_split_demo::Decision::kTrack, decision);
}

TEST(TargetDecision, CloseReliableTargetRequestsShot)
{
  const auto decision = logic_split_demo::decide(logic_split_demo::TargetInput{true, 0.90F, 25.0F}, kConfig);

  EXPECT_EQ(logic_split_demo::Decision::kRequestShot, decision);
}
