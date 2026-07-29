#include <gtest/gtest.h>

#include <cstddef>
#include <vector>

#include "antidrone_turret/actuator_model.hpp"
#include "antidrone_turret/core_controller.hpp"
#include "antidrone_turret/target_sequence.hpp"

namespace {

using ActuatorModel = antidrone_turret::ActuatorModel;
using ActuatorState = antidrone_turret::ActuatorState;
using TargetSample = antidrone_turret::TargetSample;

constexpr float kConfidenceThreshold = 0.80F;
constexpr float kMaxDistanceM = 30.0F;

core::CoreController make_controller()
{
  return core::CoreController{kConfidenceThreshold, kMaxDistanceM};
}

// Значення нижче — константи з msg/TurretStatus.msg, ServoCommand.msg і
// GimbalCommand.msg. turret_controller_node публікує enum через static_cast,
// тому нумерація мусить збігатися.
TEST(CoreControllerTest, EnumValuesMatchMessageConstants)
{
  EXPECT_EQ(static_cast<int>(core::TargetState::TARGET_NONE), 0);
  EXPECT_EQ(static_cast<int>(core::TargetState::TARGET_LOW_CONFIDENCE), 1);
  EXPECT_EQ(static_cast<int>(core::TargetState::TARGET_LOCKED), 2);

  EXPECT_EQ(static_cast<int>(core::Action::ACTION_IDLE), 0);
  EXPECT_EQ(static_cast<int>(core::Action::ACTION_TRACK), 1);

  EXPECT_EQ(static_cast<int>(core::TriggerState::TRIGGER_SKIP), 0);
  EXPECT_EQ(static_cast<int>(core::TriggerState::TRIGGER_REQUESTED), 1);
  EXPECT_EQ(static_cast<int>(core::TriggerState::TRIGGER_RELOADING), 2);

  EXPECT_EQ(static_cast<int>(core::ServoDirection::LEFT), -1);
  EXPECT_EQ(static_cast<int>(core::ServoDirection::CENTER), 0);
  EXPECT_EQ(static_cast<int>(core::ServoDirection::RIGHT), 1);

  EXPECT_EQ(static_cast<int>(core::GimbalDirection::DOWN), -1);
  EXPECT_EQ(static_cast<int>(core::GimbalDirection::CENTER), 0);
  EXPECT_EQ(static_cast<int>(core::GimbalDirection::UP), 1);
}

// Оцінка цілі: visible=false -> TARGET_NONE, ACTION_IDLE, TRIGGER_SKIP.
TEST(CoreControllerTest, InvisibleTargetStaysIdle)
{
  auto controller = make_controller();

  const auto result = controller.computeTriggerResult(TargetSample{false, 400.0F, 200.0F, 10.0F, 0.99F}, ActuatorState::kReady);

  EXPECT_EQ(result.action, core::Action::ACTION_IDLE);
  EXPECT_EQ(result.targetState, core::TargetState::TARGET_NONE);
  EXPECT_EQ(result.triggerState, core::TriggerState::TRIGGER_SKIP);

  EXPECT_EQ(result.servoCommand.servoDirection, core::ServoDirection::CENTER);
  EXPECT_FLOAT_EQ(result.servoCommand.targetX, 0.0F);
  EXPECT_FLOAT_EQ(result.servoCommand.errorX, 0.0F);
  EXPECT_EQ(result.gimbalCommand.gimbalDirection, core::GimbalDirection::CENTER);
  EXPECT_FLOAT_EQ(result.gimbalCommand.targetY, 0.0F);
  EXPECT_FLOAT_EQ(result.gimbalCommand.errorY, 0.0F);
}

// Оцінка цілі: confidence нижче порога -> ACTION_IDLE, TRIGGER_SKIP.
TEST(CoreControllerTest, LowConfidenceTargetIsReportedWithoutTracking)
{
  auto controller = make_controller();

  const auto result = controller.computeTriggerResult(TargetSample{true, 400.0F, 200.0F, 10.0F, 0.79F}, ActuatorState::kReady);

  EXPECT_EQ(result.action, core::Action::ACTION_IDLE);
  EXPECT_EQ(result.targetState, core::TargetState::TARGET_LOW_CONFIDENCE);
  EXPECT_EQ(result.triggerState, core::TriggerState::TRIGGER_SKIP);

  EXPECT_EQ(result.servoCommand.servoDirection, core::ServoDirection::CENTER);
  EXPECT_FLOAT_EQ(result.servoCommand.targetX, 0.0F);
  EXPECT_EQ(result.gimbalCommand.gimbalDirection, core::GimbalDirection::CENTER);
  EXPECT_FLOAT_EQ(result.gimbalCommand.targetY, 0.0F);
}

TEST(CoreControllerTest, ConfidenceAtThresholdIsAccepted)
{
  auto controller = make_controller();

  const auto result =
    controller.computeTriggerResult(TargetSample{true, 320.0F, 240.0F, 20.0F, kConfidenceThreshold}, ActuatorState::kReady);

  EXPECT_EQ(result.action, core::Action::ACTION_TRACK);
  EXPECT_EQ(result.targetState, core::TargetState::TARGET_LOCKED);
  EXPECT_EQ(result.triggerState, core::TriggerState::TRIGGER_REQUESTED);
}

// Команда yaw-серво: x > 320 -> RIGHT, error_x > 0.
TEST(CoreControllerTest, TargetRightOfCenterSteersServoRightWithPositiveError)
{
  auto controller = make_controller();

  const auto result = controller.computeTriggerResult(TargetSample{true, 420.0F, 240.0F, 25.0F, 0.93F}, ActuatorState::kReady);

  EXPECT_EQ(result.servoCommand.servoDirection, core::ServoDirection::RIGHT);
  EXPECT_GT(result.servoCommand.errorX, 0.0F);
  EXPECT_FLOAT_EQ(result.servoCommand.targetX, 420.0F);
  EXPECT_FLOAT_EQ(result.servoCommand.errorX, 100.0F);
}

TEST(CoreControllerTest, TargetLeftOfCenterSteersServoLeftWithNegativeError)
{
  auto controller = make_controller();

  const auto result = controller.computeTriggerResult(TargetSample{true, 200.0F, 240.0F, 25.0F, 0.93F}, ActuatorState::kReady);

  EXPECT_EQ(result.servoCommand.servoDirection, core::ServoDirection::LEFT);
  EXPECT_LT(result.servoCommand.errorX, 0.0F);
  EXPECT_FLOAT_EQ(result.servoCommand.targetX, 200.0F);
  EXPECT_FLOAT_EQ(result.servoCommand.errorX, -120.0F);
}

// Команда гімбала: y < 240 -> UP, error_y > 0 (error_y = 240 - y).
TEST(CoreControllerTest, TargetAboveCenterSteersGimbalUpWithPositiveError)
{
  auto controller = make_controller();

  const auto result = controller.computeTriggerResult(TargetSample{true, 320.0F, 180.0F, 25.0F, 0.93F}, ActuatorState::kReady);

  EXPECT_EQ(result.gimbalCommand.gimbalDirection, core::GimbalDirection::UP);
  EXPECT_GT(result.gimbalCommand.errorY, 0.0F);
  EXPECT_FLOAT_EQ(result.gimbalCommand.targetY, 180.0F);
  EXPECT_FLOAT_EQ(result.gimbalCommand.errorY, 60.0F);
}

TEST(CoreControllerTest, TargetBelowCenterSteersGimbalDownWithNegativeError)
{
  auto controller = make_controller();

  const auto result = controller.computeTriggerResult(TargetSample{true, 320.0F, 400.0F, 25.0F, 0.93F}, ActuatorState::kReady);

  EXPECT_EQ(result.gimbalCommand.gimbalDirection, core::GimbalDirection::DOWN);
  EXPECT_LT(result.gimbalCommand.errorY, 0.0F);
  EXPECT_FLOAT_EQ(result.gimbalCommand.targetY, 400.0F);
  EXPECT_FLOAT_EQ(result.gimbalCommand.errorY, -160.0F);
}

TEST(CoreControllerTest, CenteredTargetKeepsBothAxesCentered)
{
  auto controller = make_controller();

  const auto result = controller.computeTriggerResult(TargetSample{true, 320.0F, 240.0F, 25.0F, 0.93F}, ActuatorState::kReady);

  EXPECT_EQ(result.servoCommand.servoDirection, core::ServoDirection::CENTER);
  EXPECT_FLOAT_EQ(result.servoCommand.targetX, 320.0F);
  EXPECT_FLOAT_EQ(result.servoCommand.errorX, 0.0F);
  EXPECT_EQ(result.gimbalCommand.gimbalDirection, core::GimbalDirection::CENTER);
  EXPECT_FLOAT_EQ(result.gimbalCommand.targetY, 240.0F);
  EXPECT_FLOAT_EQ(result.gimbalCommand.errorY, 0.0F);
}

// Рішення щодо пострілу: близька ціль + актуатор READY -> TRIGGER_REQUESTED.
TEST(CoreControllerTest, ReadyActuatorOnCloseTargetRequestsTrigger)
{
  auto controller = make_controller();

  const auto result = controller.computeTriggerResult(TargetSample{true, 360.0F, 260.0F, 15.0F, 0.95F}, ActuatorState::kReady);

  EXPECT_EQ(result.action, core::Action::ACTION_TRACK);
  EXPECT_EQ(result.targetState, core::TargetState::TARGET_LOCKED);
  EXPECT_EQ(result.triggerState, core::TriggerState::TRIGGER_REQUESTED);
}

// Рішення щодо пострілу: близька ціль + актуатор RELOADING ->
// TRIGGER_RELOADING.
TEST(CoreControllerTest, ReloadingActuatorOnCloseTargetDefersTrigger)
{
  auto controller = make_controller();

  const auto result = controller.computeTriggerResult(TargetSample{true, 360.0F, 260.0F, 15.0F, 0.95F}, ActuatorState::kReloading);

  EXPECT_EQ(result.action, core::Action::ACTION_TRACK);
  EXPECT_EQ(result.targetState, core::TargetState::TARGET_LOCKED);
  EXPECT_EQ(result.triggerState, core::TriggerState::TRIGGER_RELOADING);

  EXPECT_EQ(result.servoCommand.servoDirection, core::ServoDirection::RIGHT);
  EXPECT_FLOAT_EQ(result.servoCommand.errorX, 40.0F);
  EXPECT_EQ(result.gimbalCommand.gimbalDirection, core::GimbalDirection::DOWN);
  EXPECT_FLOAT_EQ(result.gimbalCommand.errorY, -20.0F);
}

TEST(CoreControllerTest, TargetAtMaxDistanceStillAllowsTrigger)
{
  auto controller = make_controller();

  const auto result = controller.computeTriggerResult(TargetSample{true, 320.0F, 240.0F, kMaxDistanceM, 0.95F}, ActuatorState::kReady);

  EXPECT_EQ(result.targetState, core::TargetState::TARGET_LOCKED);
  EXPECT_EQ(result.triggerState, core::TriggerState::TRIGGER_REQUESTED);
}

// Статус контролера: далека коректна ціль -> TARGET_LOCKED, ACTION_TRACK,
// TRIGGER_SKIP.
TEST(CoreControllerTest, FarValidTargetIsLockedAndTrackedWithoutTrigger)
{
  auto controller = make_controller();

  const auto result = controller.computeTriggerResult(TargetSample{true, 400.0F, 200.0F, 30.5F, 0.95F}, ActuatorState::kReady);

  EXPECT_EQ(result.targetState, core::TargetState::TARGET_LOCKED);
  EXPECT_EQ(result.action, core::Action::ACTION_TRACK);
  EXPECT_EQ(result.triggerState, core::TriggerState::TRIGGER_SKIP);

  EXPECT_EQ(result.servoCommand.servoDirection, core::ServoDirection::RIGHT);
  EXPECT_FLOAT_EQ(result.servoCommand.targetX, 400.0F);
  EXPECT_FLOAT_EQ(result.servoCommand.errorX, 80.0F);
  EXPECT_EQ(result.gimbalCommand.gimbalDirection, core::GimbalDirection::UP);
  EXPECT_FLOAT_EQ(result.gimbalCommand.targetY, 200.0F);
  EXPECT_FLOAT_EQ(result.gimbalCommand.errorY, 40.0F);
}

TEST(CoreControllerTest, DefaultSequenceLocksEveryConfidentSample)
{
  auto controller = make_controller();

  std::vector<core::TargetState> target_states;
  std::vector<core::Action> actions;
  for (const auto &sample : antidrone_turret::default_target_samples()) {
    const auto result = controller.computeTriggerResult(sample, ActuatorState::kReady);
    target_states.push_back(result.targetState);
    actions.push_back(result.action);
  }

  ASSERT_EQ(target_states.size(), 6U);

  EXPECT_EQ(actions[0], core::Action::ACTION_IDLE);
  EXPECT_EQ(target_states[0], core::TargetState::TARGET_LOW_CONFIDENCE);

  for (std::size_t index = 1; index < target_states.size(); ++index) {
    EXPECT_EQ(actions[index], core::Action::ACTION_TRACK) << "sample " << index;
    EXPECT_EQ(target_states[index], core::TargetState::TARGET_LOCKED) << "sample " << index;
  }
}

TEST(CoreControllerTest, DefaultSequenceTriggersOnceUntilActuatorReloads)
{
  auto controller = make_controller();
  auto actuator = ActuatorModel{};

  std::vector<core::TriggerState> trigger_states;
  for (const auto &sample : antidrone_turret::default_target_samples()) {
    const auto result = controller.computeTriggerResult(sample, actuator.state());
    trigger_states.push_back(result.triggerState);

    if (result.triggerState == core::TriggerState::TRIGGER_REQUESTED) {
      EXPECT_TRUE(actuator.trigger().accepted);
    }
  }

  ASSERT_EQ(trigger_states.size(), 6U);
  for (std::size_t index = 0; index < 4U; ++index) {
    EXPECT_EQ(trigger_states[index], core::TriggerState::TRIGGER_SKIP) << "sample " << index;
  }
  EXPECT_EQ(trigger_states[4], core::TriggerState::TRIGGER_REQUESTED);
  EXPECT_EQ(trigger_states[5], core::TriggerState::TRIGGER_RELOADING);
  EXPECT_EQ(actuator.trigger_count(), 1U);
}

TEST(CoreControllerTest, ReloadedActuatorRequestsTriggerAgain)
{
  auto controller = make_controller();
  auto actuator = ActuatorModel{};
  const auto locked_target = TargetSample{true, 420.0F, 180.0F, 25.0F, 0.93F};

  const auto first = controller.computeTriggerResult(locked_target, actuator.state());
  ASSERT_EQ(first.triggerState, core::TriggerState::TRIGGER_REQUESTED);
  ASSERT_TRUE(actuator.trigger().accepted);

  const auto during_reload = controller.computeTriggerResult(locked_target, actuator.state());
  EXPECT_EQ(during_reload.triggerState, core::TriggerState::TRIGGER_RELOADING);

  actuator.mark_ready();
  const auto after_reload = controller.computeTriggerResult(locked_target, actuator.state());
  EXPECT_EQ(after_reload.triggerState, core::TriggerState::TRIGGER_REQUESTED);
  EXPECT_TRUE(actuator.trigger().accepted);
  EXPECT_EQ(actuator.trigger_count(), 2U);
}

}  // namespace
