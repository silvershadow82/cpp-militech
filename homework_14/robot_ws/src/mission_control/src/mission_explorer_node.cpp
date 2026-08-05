#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "underground_world/msg/local_scan.hpp"
#include "underground_world/msg/move_command.hpp"
#include "underground_world/msg/robot_metrics.hpp"
#include "underground_world/msg/robot_result.hpp"
#include "underground_world/msg/student_status.hpp"
#include "underground_world/srv/payload_trigger.hpp"
#include "underground_world/state_qos.hpp"

#include "mission_control/explorer.hpp"

namespace {

using underground_world::msg::LocalScan;
using underground_world::msg::MoveCommand;
using underground_world::msg::RobotMetrics;
using underground_world::msg::RobotResult;
using underground_world::msg::StudentStatus;
using underground_world::srv::PayloadTrigger;

constexpr auto scanTopic = "/robot/local_scan";
constexpr auto metricsTopic = "/robot/metrics";
constexpr auto resultTopic = "/robot/result";
constexpr auto moveTopic = "/robot/cmd_move";
constexpr auto statusTopic = "/student/status";
constexpr auto triggerServiceName = "/payload/trigger";

constexpr auto watchdogPeriod = std::chrono::milliseconds{100};
constexpr int maxCommandResends = 3;

// Скільки тіків чекати на /robot/result=SUCCESS після того, як алгоритм уже
// вважає карту дослідженою. Захист від тихого зависання у стані DONE.
constexpr int maxDoneTicks = 50;

mission_control::Observation toObservation(const LocalScan& scan)
{
  mission_control::Observation observation;
  observation.robot = mission_control::Cell{scan.robot_x, scan.robot_y};
  observation.cells.reserve(scan.cells.size());

  for (const auto& cell : scan.cells) {
    mission_control::ObservedCell observed;
    observed.cell = mission_control::Cell{cell.x, cell.y};
    observed.state = mission_control::cellStateFromType(cell.cell_type);
    observed.contactId = cell.contact_id;
    observation.cells.push_back(observed);
  }

  return observation;
}

std::uint8_t toMoveCommand(const mission_control::Direction direction)
{
  switch (direction) {
    case mission_control::Direction::Up:
      return MoveCommand::UP;
    case mission_control::Direction::Down:
      return MoveCommand::DOWN;
    case mission_control::Direction::Left:
      return MoveCommand::LEFT;
    case mission_control::Direction::Right:
      return MoveCommand::RIGHT;
  }

  return MoveCommand::UP;
}

class MissionExplorerNode final : public rclcpp::Node {
public:
  MissionExplorerNode()
    : Node("mission_explorer_node")
  {
    this->startDelay = readDuration("start_delay_ms", 1500);
    this->stateTimeout = readDuration("state_timeout_ms", 2000);
    this->triggerTimeout = readDuration("trigger_timeout_ms", 1000);
    this->triggerMaxRetries = declare_parameter<int>("trigger_max_retries", 3);
    const auto statusPeriod = std::chrono::milliseconds{declare_parameter<int>("status_period_ms", 200)};

    // Профіль світу: без нього Discovery не змечить writer і reader,
    // а стартовий стан, опублікований до нашого старту, буде втрачений.
    const auto stateQos = underground_world::make_state_qos();
    const auto eventQos = rclcpp::QoS{10};
    const auto statusQos = rclcpp::QoS{rclcpp::KeepLast(10)}.reliable().transient_local();

    this->movePublisher = create_publisher<MoveCommand>(moveTopic, eventQos);
    this->statusPublisher = create_publisher<StudentStatus>(statusTopic, statusQos);

    this->scanSubscription = create_subscription<LocalScan>(scanTopic, stateQos, [this](const LocalScan::SharedPtr msg) { onScan(*msg); });
    this->resultSubscription =
      create_subscription<RobotResult>(resultTopic, stateQos, [this](const RobotResult::SharedPtr msg) { onResult(*msg); });
    this->metricsSubscription =
      create_subscription<RobotMetrics>(metricsTopic, stateQos, [this](const RobotMetrics::SharedPtr msg) { this->metrics = *msg; });

    this->triggerClient = create_client<PayloadTrigger>(triggerServiceName);

    // Пауза перед першою командою дає ros2 bag record під'єднатись до всіх
    // топіків; карта тим часом уже накопичується зі стартового scan.
    this->startTimer = create_wall_timer(this->startDelay, [this]() {
      this->startTimer->cancel();
      this->started = true;
      act();
    });
    this->watchdogTimer = create_wall_timer(watchdogPeriod, [this]() { onWatchdog(); });
    this->statusTimer = create_wall_timer(statusPeriod, [this]() { publishStatus(); });

    RCLCPP_INFO(get_logger(), "mission explorer ready, first command in %ld ms", this->startDelay.count());
  }

private:
  std::chrono::milliseconds readDuration(const std::string& name, const int fallback)
  {
    const auto value = declare_parameter<int>(name, fallback);
    return std::chrono::milliseconds{value < 1 ? 1 : value};
  }

  void onScan(const LocalScan& scan)
  {
    this->explorer.observe(toObservation(scan));
    this->awaitingState = false;
    this->commandResends = 0;

    // Світ показав контакт як оброблений - запит вважається завершеним.
    if (this->pendingTrigger.has_value() && !this->explorer.isEngaged(*this->pendingTrigger)) {
      this->pendingTrigger.reset();
      this->triggerRetries = 0;
    }

    act();
  }

  void onResult(const RobotResult& result)
  {
    if (result.mission_result == "SUCCESS") {
      finish(StudentStatus::DONE, "world reported SUCCESS");
    }
    else if (result.mission_result == "FAILED_MAX_STEPS") {
      finish(StudentStatus::FAILED, "move budget exhausted");
    }
  }

  void act()
  {
    if (this->finished || !this->started) {
      return;
    }

    const auto decision = this->explorer.decide();

    switch (decision.kind) {
      case mission_control::Decision::Kind::Engage:
        sendTrigger(decision.contact);
        break;
      case mission_control::Decision::Kind::Move:
        sendMove(decision.direction);
        break;
      case mission_control::Decision::Kind::Wait:
        this->status = this->pendingTrigger.has_value() ? StudentStatus::ENGAGING : StudentStatus::EXPLORING;
        break;
      case mission_control::Decision::Kind::Done:
        this->status = StudentStatus::DONE;
        break;
      case mission_control::Decision::Kind::Failed:
        finish(StudentStatus::FAILED, "no reachable frontier left");
        break;
    }

    publishStatus();
  }

  void sendMove(const mission_control::Direction direction)
  {
    MoveCommand command;
    command.direction = toMoveCommand(direction);
    this->movePublisher->publish(command);

    this->lastCommand = command;
    this->awaitingState = true;
    this->lastCommandTime = now();
    this->status = StudentStatus::EXPLORING;

    RCLCPP_INFO(get_logger(), "move %s", std::string(mission_control::to_string(direction)).c_str());
  }

  void sendTrigger(const mission_control::VisibleContact& contact)
  {
    if (!this->triggerClient->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "payload service not ready yet");
      return;
    }

    auto request = std::make_shared<PayloadTrigger::Request>();
    request->contact_id = contact.contactId;
    request->x = contact.cell.x;
    request->y = contact.cell.y;

    // Позначаємо контакт до відправки: доки запит у роботі, decide() поверне
    // Wait, і робот не зрушить з клітинки, з якої контакт видимий.
    this->explorer.markEngaged(contact.contactId);
    this->pendingTrigger = contact.contactId;
    this->triggerSentTime = now();
    this->status = StudentStatus::ENGAGING;

    this->triggerClient->async_send_request(request, [this, id = contact.contactId](rclcpp::Client<PayloadTrigger>::SharedFuture future) {
      onTriggerResponse(id, future.get());
    });

    RCLCPP_INFO(get_logger(), "trigger contact_id=%d at (%d,%d)", contact.contactId, contact.cell.x, contact.cell.y);
  }

  void onTriggerResponse(const int contact_id, const std::shared_ptr<PayloadTrigger::Response> response)
  {
    if (response->accepted) {
      return;
    }

    RCLCPP_WARN(get_logger(), "trigger rejected for contact %d: %s", contact_id, response->reason.c_str());
    this->explorer.forgetEngaged(contact_id);
    this->pendingTrigger.reset();
    act();
  }

  void onWatchdog()
  {
    if (this->summaryPending) {
      this->summaryPending = false;
      logSummary();
    }

    if (!this->started || this->finished) {
      return;
    }

    if (this->pendingTrigger.has_value() && (now() - this->triggerSentTime) > rclcpp::Duration(this->triggerTimeout)) {
      const auto contact_id = *this->pendingTrigger;
      this->explorer.forgetEngaged(contact_id);
      this->pendingTrigger.reset();
      ++this->triggerRetries;

      if (this->triggerRetries > this->triggerMaxRetries) {
        finish(StudentStatus::FAILED, "payload trigger was never confirmed");
        return;
      }

      RCLCPP_WARN(get_logger(), "trigger timeout for contact %d, retry %d", contact_id, this->triggerRetries);
      act();
      return;
    }

    if (this->awaitingState && (now() - this->lastCommandTime) > rclcpp::Duration(this->stateTimeout)) {
      ++this->commandResends;

      if (this->commandResends > maxCommandResends) {
        finish(StudentStatus::FAILED, "world stopped publishing state");
        return;
      }

      RCLCPP_WARN(get_logger(), "no world state after move, resend %d", this->commandResends);
      this->movePublisher->publish(this->lastCommand);
      this->lastCommandTime = now();
      return;
    }

    if (this->status == StudentStatus::DONE) {
      ++this->doneTicks;
      if (this->doneTicks > maxDoneTicks) {
        finish(StudentStatus::FAILED, "exploration complete but world never reported SUCCESS");
        return;
      }
    }

    if (!this->awaitingState && !this->pendingTrigger.has_value()) {
      act();
    }
  }

  void finish(const std::uint8_t state, const std::string& reason)
  {
    if (this->finished) {
      return;
    }

    this->finished = true;
    this->status = state;
    this->finishReason = reason;
    publishStatus();

    // Підсумок друкуємо на наступному тіку: /robot/metrics і /robot/result
    // приходять різними підписками, і executor може віддати result першим -
    // тоді знімок метрик у цей момент відстає на одне повідомлення.
    this->summaryPending = true;
  }

  void logSummary()
  {
    RCLCPP_INFO(get_logger(),
                "mission finished: %s (steps=%u invalid_moves=%u contacts=%u/%u coverage=%.1f%%)",
                this->finishReason.c_str(),
                this->metrics.steps_taken,
                this->metrics.invalid_moves,
                this->metrics.contacts_down,
                this->metrics.contacts_seen,
                static_cast<double>(this->metrics.map_coverage_percent));
  }

  void publishStatus()
  {
    StudentStatus message;
    message.state = this->status;
    this->statusPublisher->publish(message);
  }

  mission_control::Explorer explorer;
  RobotMetrics metrics;
  MoveCommand lastCommand;

  std::chrono::milliseconds startDelay{1500};
  std::chrono::milliseconds stateTimeout{2000};
  std::chrono::milliseconds triggerTimeout{1000};
  int triggerMaxRetries = 3;

  std::uint8_t status = StudentStatus::EXPLORING;
  std::string finishReason;
  bool summaryPending = false;
  std::optional<int> pendingTrigger;
  rclcpp::Time triggerSentTime;
  rclcpp::Time lastCommandTime;
  int triggerRetries = 0;
  int commandResends = 0;
  int doneTicks = 0;
  bool started = false;
  bool awaitingState = false;
  bool finished = false;

  rclcpp::Publisher<MoveCommand>::SharedPtr movePublisher;
  rclcpp::Publisher<StudentStatus>::SharedPtr statusPublisher;
  rclcpp::Subscription<LocalScan>::SharedPtr scanSubscription;
  rclcpp::Subscription<RobotResult>::SharedPtr resultSubscription;
  rclcpp::Subscription<RobotMetrics>::SharedPtr metricsSubscription;
  rclcpp::Client<PayloadTrigger>::SharedPtr triggerClient;
  rclcpp::TimerBase::SharedPtr startTimer;
  rclcpp::TimerBase::SharedPtr watchdogTimer;
  rclcpp::TimerBase::SharedPtr statusTimer;
};

}  // namespace

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionExplorerNode>());
  rclcpp::shutdown();
  return 0;
}
