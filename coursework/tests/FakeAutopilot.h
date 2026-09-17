#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <common/mavlink.h>
#include <optional>
#include <span>
#include <thread>
#include <vector>

#include "MavlinkTestSupport.h"
#include "follow/core/Types.h"
#include "follow/mavlink/Links.h"
#include "follow/sim/KinematicVehicle.h"

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

private:
  using Clock = std::chrono::steady_clock;

  void loop()
  {
    const auto period = std::chrono::milliseconds{10};
    Clock::time_point next = Clock::now();
    std::optional<Clock::time_point> connected;
    std::optional<core::VelocityCmd> setpoint;
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
      if (tick % 100 == 0) {
        this->send(this->fc.heartbeat(guided ? core::kModeGuided : core::kModeLoiter, true));
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
  bool receiveSetpoint(std::optional<core::VelocityCmd>& setpoint)
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
          setpoint = core::VelocityCmd{.vx = target.vx, .yawRate = target.yaw_rate};
        }
      }
    }
    return any;
  }

  void send(const std::vector<uint8_t>& bytes) { this->link.send(std::span<const uint8_t>(bytes.data(), bytes.size())); }

  mavlink::UdpLink link;
  sim::KinematicVehicle vehicle;
  double engageAfterS;
  Peer fc{1, 1};
  mavlink_message_t rxBuffer{};
  mavlink_status_t rxStatus{};
  std::atomic<bool> stopRequested{false};
  std::thread thread;
};

}  // namespace follow::test
