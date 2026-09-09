#pragma once

#include "comms/SocketLink.h"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>

#include <common/mavlink.h>

// double, а не float: на 5.0e8 (градуси * 1e7)
constexpr double LAT0 = 50.4501;
constexpr double LON0 = 30.5234;
constexpr double COEFF = 111320.0;

constexpr size_t MAX_COMPONENTS = 256;

constexpr uint16_t DROP_COMMAND = MAV_CMD_USER_1;
constexpr std::chrono::milliseconds DROP_RETRY_PERIOD{2000};

constexpr int DROP_MAX_ATTEMPTS = 5;

namespace comms {

struct GlobalPosition {
  float x{0.0};
  float y{0.0};
  float z{0.0};
  float speed{0.0};
  float dir{0.0};
  uint32_t timeBootMs{0};

  double lat() const { return LAT0 + (y / COEFF); }
  double lon() const { return LON0 + (x / (COEFF * std::cos(LAT0 * M_PI / 180))); }

  float velNorth() const { return speed * std::sin(dir); }
  float velEast() const { return speed * std::cos(dir); }
  float headingDeg() const
  {
    float deg = static_cast<float>(std::atan2(velEast(), velNorth()) * 180.0 / M_PI);
    return deg < 0.0F ? deg + 360.0F : deg;
  }
};

struct Attitude {
  float pitch{0.0};
  float roll{0.0};
  float yaw{0.0};
  uint32_t timeBootMs{0};
};

class MavLink {
private:
  std::unique_ptr<comms::SocketLink> link;
  std::mutex libMtx;

  mutable std::mutex mtx;
  std::array<uint64_t, MAX_COMPONENTS> lastHeartbeat{};
  uint64_t rxCount{0};

  struct PendingDrop {
    float lat{0.0F};
    float lon{0.0F};
    float alt{0.0F};
    int attempts{0};
    bool acked{false};
    bool gaveUpLogged{false};
    std::chrono::steady_clock::time_point lastSend{};
  };
  std::optional<PendingDrop> pendingDrop{};

  mutable uint32_t lastTimeBootMs{0};
  uint32_t monotonicBootMs(uint32_t simTimeMs) const;

public:
  explicit MavLink(std::unique_ptr<comms::SocketLink> link)
    : link(std::move(link))
  {
  }

  void on_message(const mavlink_message_t *m);
  void send_msg(const mavlink_message_t *m);
  int rx_poll();
  void send_heartbeat();
  void send_attitude(const Attitude &a);
  void send_global_position(const GlobalPosition &gp);

  void requestDrop(float lat, float lon, float alt);
  void serviceDrop();
  bool dropAcked() const;
  int dropAttempts() const;
  // true, якщо спроби вичерпано, а ACK так і не прийшов.
  bool dropGaveUp() const;
  // true, якщо скид завершено: отримано ACK або вичерпано всі спроби.
  bool isDropResolved() const { return dropAcked() || dropGaveUp(); }

  bool isOpen() const { return this->link && this->link->isOpen(); }
  bool hasPeer() const { return this->link && this->link->hasPeer(); }

  uint64_t receivedMessages() const;
  // Час (ms since epoch) останнього HEARTBEAT від компонента, 0 якщо не було.
  uint64_t lastHeartbeatFrom(uint8_t compid) const;
};
}  // namespace comms
