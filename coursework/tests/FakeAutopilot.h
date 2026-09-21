#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <common/mavlink.h>
#include <mutex>
#include <optional>
#include <span>
#include <thread>
#include <vector>

#include "MavlinkTestSupport.h"
#include "Types.h"
#include "comms/SocketLink.h"
#include "sim/KinematicVehicle.h"

namespace follow::test {

// A stand-in ArduCopter over UDP for end-to-end tests: a KinematicVehicle hovering 2 m up that
// streams HEARTBEAT, ATTITUDE and LOCAL_POSITION_NED, switches LOITER -> GUIDED `engageAfterS`
// after the app's first datagram, and flies the velocity setpoints it receives while in GUIDED.
class FakeAutopilot {
public:
  explicit FakeAutopilot(double engageAfterS)
    : link(0)
    , vehicle(sim::Pose{.positionNed = {0.0, 0.0, -2.0}}, sim::KinematicVehicleConfig{})
    , engageAfterS(engageAfterS)
  {
  }

  ~FakeAutopilot() { this->stop(); }

  int port() const { return this->link.localPort(); }

  void start()
  {
    this->thread = std::thread([this] { this->loop(); });
  }

  void stop()
  {
    this->stopRequested = true;
    if (this->thread.joinable()) {
      this->thread.join();
    }
  }

  // The most recent setpoint received, for tests asserting what the FC was left holding.
  std::optional<models::VelocityCmd> lastSetpoint() const
  {
    std::lock_guard<std::mutex> lock(this->setpointMutex);
    return this->lastReceived;
  }

private:
  using Clock = std::chrono::steady_clock;

  void loop()
  {
    const auto period = std::chrono::milliseconds{10};
    Clock::time_point next = Clock::now();
    std::optional<Clock::time_point> connected;
    std::optional<bool> announced;  // the mode the last heartbeat reported, so a change can be sent at once
    std::optional<models::VelocityCmd> setpoint;
    Clock::time_point setpointTime{};
    for (int tick = 0; !this->stopRequested; ++tick) {
      Clock::time_point now = Clock::now();
      if (this->receiveSetpoint(setpoint)) {
        setpointTime = now;
        connected = connected.value_or(now);
      }
      bool guided = connected && now - *connected >= std::chrono::duration<double>(this->engageAfterS);
      // GUID_TIMEOUT: stop when setpoints stop for 3 s.
      bool fresh = setpoint && now - setpointTime < std::chrono::seconds{3};
      this->vehicle.step(guided && fresh ? setpoint : std::nullopt, 0.01);

      const sim::Pose& pose = this->vehicle.pose();
      // A real FC broadcasts HEARTBEAT continuously; over UDP this one cannot until the app has
      // spoken first, and at 1 Hz it would then quantise both the first contact and the
      // LOITER -> GUIDED change to a one-second grid. An app waiting on either would leave NoFc,
      // and lock, at a time that depends on where in the heartbeat period the run happened to
      // start -- a race, and a slow one. Announce both the moment they happen.
      //
      // `announced` records only what a listener could actually have heard: before the first
      // datagram arrives there is nowhere to send, so those heartbeats must not count as announced,
      // or first contact finds guided == announced and says nothing until the next 1 Hz tick.
      if (tick % 100 == 0 || (connected && guided != announced)) {
        if (connected) {
          announced = guided;
        }
        this->send(this->fc.heartbeat(guided ? models::kModeGuided : models::kModeLoiter, true));
      }
      if (tick % 5 == 0) {
        this->send(this->fc.attitude(static_cast<float>(pose.roll), static_cast<float>(pose.pitch), static_cast<float>(pose.yaw)));
      }
      if (tick % 10 == 0) {
        double speed = this->vehicle.forwardSpeed();
        this->send(this->fc.localPosition(static_cast<float>(pose.positionNed.x),
                                          static_cast<float>(pose.positionNed.y),
                                          static_cast<float>(pose.positionNed.z),
                                          static_cast<float>(speed * std::cos(pose.yaw)),
                                          static_cast<float>(speed * std::sin(pose.yaw)),
                                          0.0F));
      }
      next += period;
      std::this_thread::sleep_until(next);
    }
  }

  // Reads all pending datagrams; true if any arrived (the app is connected). Updates `setpoint`.
  bool receiveSetpoint(std::optional<models::VelocityCmd>& setpoint)
  {
    bool any = false;
    std::array<uint8_t, 512> buffer{};
    for (int n = this->link.receive(buffer); n > 0; n = this->link.receive(buffer)) {
      any = true;
      for (int i = 0; i < n; ++i) {
        mavlink_message_t message{};
        mavlink_status_t status{};
        if (mavlink_frame_char_buffer(&this->rxBuffer, &this->rxStatus, buffer[static_cast<size_t>(i)], &message, &status) ==
              MAVLINK_FRAMING_OK &&
            message.msgid == MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED) {
          mavlink_set_position_target_local_ned_t target{};
          mavlink_msg_set_position_target_local_ned_decode(&message, &target);
          setpoint = models::VelocityCmd{.vx = target.vx, .yawRate = target.yaw_rate};
          std::lock_guard<std::mutex> lock(this->setpointMutex);
          this->lastReceived = setpoint;
        }
      }
    }
    return any;
  }

  void send(const std::vector<uint8_t>& bytes) { this->link.send(std::span<const uint8_t>(bytes.data(), bytes.size())); }

  comms::SocketLink link;
  sim::KinematicVehicle vehicle;
  double engageAfterS;
  Peer fc{1, 1};
  mavlink_message_t rxBuffer{};
  mavlink_status_t rxStatus{};
  std::atomic<bool> stopRequested{false};
  mutable std::mutex setpointMutex;
  std::optional<models::VelocityCmd> lastReceived{};
  std::thread thread;
};

}  // namespace follow::test
