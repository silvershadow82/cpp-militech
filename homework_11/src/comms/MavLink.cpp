#include "comms/MavLink.h"
#include "comms/SocketLink.h"
#include "debug.h"

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <sys/socket.h>

#include <common/mavlink.h>

#define SYS_ID 1
#define SRC_COMP_ID MAV_COMP_ID_AUTOPILOT1
#define TARGET_COMP_ID MAV_COMP_ID_AUTOPILOT1

namespace comms {

namespace {
constexpr int MAX_DATAGRAMS_PER_POLL = 256;
}  // namespace

uint64_t now_ms()
{
  auto now = std::chrono::system_clock::now();
  auto duration = now.time_since_epoch();
  return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
}

void MavLink::on_message(const mavlink_message_t *m)
{
  switch (m->msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT: {
      std::lock_guard<std::mutex> lock(this->mtx);
      this->lastHeartbeat[m->compid] = now_ms();
      break;
    }
    case MAVLINK_MSG_ID_COMMAND_ACK: {
      mavlink_command_ack_t ack{};
      mavlink_msg_command_ack_decode(m, &ack);
      if (ack.command == DROP_COMMAND) {
        std::lock_guard<std::mutex> lock(this->mtx);
        if (this->pendingDrop.has_value()) {
          this->pendingDrop->acked = true;
          LOG("MAVLink drop ACK after " << this->pendingDrop->attempts << " attempt(s), result=" << static_cast<int>(ack.result));
        }
      }
      break;
    }
    default:
      break;
  }

  std::lock_guard<std::mutex> lock(this->mtx);
  ++this->rxCount;
}

void MavLink::send_msg(const mavlink_message_t *m)
{
  if (!this->link) {
    return;
  }
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  const uint16_t len = mavlink_msg_to_send_buffer(buf, m);
  this->link->send(buf, len);
}

int MavLink::rx_poll()
{
  if (!this->link) {
    return 0;
  }

  uint8_t in[MAVLINK_MAX_PACKET_LEN];
  mavlink_message_t msg;
  mavlink_status_t st;
  int parsed = 0;

  // Сокет неблокуючий: читаємо, поки в черзі є датаграми, потім виходимо.
  for (int datagram = 0; datagram < MAX_DATAGRAMS_PER_POLL; ++datagram) {
    int n = this->link->receive(in, sizeof(in));
    if (n <= 0) {
      break;  // EAGAIN/EWOULDBLOCK - черга порожня
    }
    for (int i = 0; i < n; ++i) {
      bool complete = false;
      {
        std::lock_guard<std::mutex> lock(this->libMtx);
        complete = mavlink_parse_char(MAVLINK_COMM_0, in[i], &msg, &st) != 0;
      }
      if (complete) {
        on_message(&msg);
        ++parsed;
      }
    }
  }

  return parsed;
}

void MavLink::send_heartbeat()
{  // у таймері 1 Гц
  mavlink_message_t msg;
  {
    std::lock_guard<std::mutex> lock(this->libMtx);
    mavlink_msg_heartbeat_pack(SYS_ID,
                               SRC_COMP_ID,  // наш власний component id, не адресата
                               &msg,
                               MAV_TYPE_QUADROTOR,     // хто ми
                               MAV_AUTOPILOT_GENERIC,  // INVALID = "не апарат", QGC такий борт не показує
                               0,
                               0,
                               MAV_STATE_ACTIVE);  // режим, стан
  }
  this->send_msg(&msg);
}

void MavLink::send_attitude(const Attitude &a)
{
  mavlink_message_t msg;
  {
    std::lock_guard<std::mutex> lock(this->libMtx);
    mavlink_msg_attitude_pack(SYS_ID, SRC_COMP_ID, &msg, this->bootMs(), a.roll, a.pitch, a.yaw, 0.0F, 0.0F, 0.0F);
  }
  this->send_msg(&msg);
}

void MavLink::send_global_position(const GlobalPosition &gp)
{
  mavlink_message_t msg;
  const int32_t lat = static_cast<int32_t>(gp.lat() * 1e7);
  const int32_t lon = static_cast<int32_t>(gp.lon() * 1e7);
  const int32_t alt = static_cast<int32_t>(gp.z * 1000.0F);
  const int16_t vx = static_cast<int16_t>(gp.velNorth() * 100.0F);  // північ, см/с
  const int16_t vy = static_cast<int16_t>(gp.velEast() * 100.0F);   // схід, см/с
  const uint16_t hdg = static_cast<uint16_t>(gp.headingDeg() * 100.0F);
  {
    std::lock_guard<std::mutex> lock(this->libMtx);
    mavlink_msg_global_position_int_pack(SYS_ID, SRC_COMP_ID, &msg, this->bootMs(), lat, lon, alt, alt, vx, vy, 0, hdg);
  }
  this->send_msg(&msg);
}

uint32_t MavLink::bootMs() const
{
  const auto elapsed = std::chrono::steady_clock::now() - this->bootTime;
  return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count());
}

void MavLink::requestDrop(float lat, float lon, float alt)
{
  std::lock_guard<std::mutex> lock(this->mtx);
  if (this->pendingDrop.has_value()) {
    return;  // параметри вже зафіксовані - повтори мусять бути ідентичними
  }
  PendingDrop drop{};
  drop.lat = lat;
  drop.lon = lon;
  drop.alt = alt;
  this->pendingDrop = drop;
  LOG("MAVLink drop requested: lat=" << lat << " lon=" << lon << " alt=" << alt);
}

void MavLink::serviceDrop()
{
  float lat = 0.0F;
  float lon = 0.0F;
  float alt = 0.0F;
  int attempt = 0;

  {
    std::lock_guard<std::mutex> lock(this->mtx);
    if (!this->pendingDrop.has_value() || this->pendingDrop->acked) {
      return;  // після ACK чекер вимагає повної тиші
    }

    const auto now = std::chrono::steady_clock::now();
    if (this->pendingDrop->attempts > 0 && (now - this->pendingDrop->lastSend) < DROP_RETRY_PERIOD) {
      return;
    }

    this->pendingDrop->lastSend = now;
    attempt = ++this->pendingDrop->attempts;
    lat = this->pendingDrop->lat;
    lon = this->pendingDrop->lon;
    alt = this->pendingDrop->alt;
  }

  mavlink_message_t msg;
  {
    std::lock_guard<std::mutex> lock(this->libMtx);
    mavlink_msg_command_long_pack(SYS_ID,
                                  SRC_COMP_ID,
                                  &msg,
                                  SYS_ID,
                                  TARGET_COMP_ID,
                                  DROP_COMMAND,
                                  static_cast<uint8_t>(attempt - 1),  // confirmation
                                  0.0F,
                                  0.0F,
                                  0.0F,
                                  0.0F,
                                  lat,
                                  lon,
                                  alt);
  }
  this->send_msg(&msg);
  LOG("MAVLink drop command attempt " << attempt << " (lat=" << lat << " lon=" << lon << " alt=" << alt << ")");
}

bool MavLink::dropAcked() const
{
  std::lock_guard<std::mutex> lock(this->mtx);
  return this->pendingDrop.has_value() && this->pendingDrop->acked;
}

int MavLink::dropAttempts() const
{
  std::lock_guard<std::mutex> lock(this->mtx);
  return this->pendingDrop.has_value() ? this->pendingDrop->attempts : 0;
}

uint64_t MavLink::receivedMessages() const
{
  std::lock_guard<std::mutex> lock(this->mtx);
  return this->rxCount;
}

uint64_t MavLink::lastHeartbeatFrom(uint8_t compid) const
{
  std::lock_guard<std::mutex> lock(this->mtx);
  return this->lastHeartbeat[compid];
}

}  // namespace comms
