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

// double, а не float: на 5.0e8 (градуси * 1e7) крок float - близько 32 одиниць,
// тобто ~3 м, і чекер сипле "позиція не узгоджена зі швидкістю".
constexpr double LAT0 = 50.4501;
constexpr double LON0 = 30.5234;
constexpr double COEFF = 111320.0;

// compid у MAVLink - це uint8_t, тож таблиця "останній HEARTBEAT" має покривати
// весь діапазон 0..255. Менший розмір давав би вихід за межі масиву.
constexpr size_t MAX_COMPONENTS = 256;

// Скид передається як COMMAND_LONG з MAV_CMD_USER_1: param5/6/7 = lat/lon/alt.
// Команду треба повторювати, доки не прийде COMMAND_ACK (чекер навмисно губить
// першу спробу), причому з незмінними параметрами, і замовкнути після ACK.
constexpr uint16_t DROP_COMMAND = MAV_CMD_USER_1;
constexpr std::chrono::milliseconds DROP_RETRY_PERIOD{2000};

namespace comms {

// Позиція у локальній рамі drone_link: x - на схід, y - на північ, dir - курс
// (0 = схід, проти годинникової стрілки), як у PKT_TELEMETRY.
struct GlobalPosition {
  float x{0.0};
  float y{0.0};
  float z{0.0};
  float speed{0.0};
  float dir{0.0};

  double lat() const { return LAT0 + (y / COEFF); }
  double lon() const { return LON0 + (x / (COEFF * std::cos(LAT0 * M_PI / 180))); }

  // Складові швидкості у рамі MAVLink: північ і схід, м/с.
  float velNorth() const { return speed * std::sin(dir); }
  float velEast() const { return speed * std::cos(dir); }
  // Курс у градусах: 0 = північ, за годинниковою стрілкою.
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
};

class MavLink {
private:
  std::unique_ptr<comms::SocketLink> link;

  // Бібліотека MAVLink тримає глобальний per-channel стан (seq лічильник,
  // буфер парсера) для MAVLINK_COMM_0. Потік rx і потік heartbeat чіпають його
  // одночасно, тому всі виклики mavlink_* серіалізуються цим м'ютексом.
  std::mutex libMtx;

  mutable std::mutex mtx;
  std::array<uint64_t, MAX_COMPONENTS> lastHeartbeat{};
  uint64_t rxCount{0};

  // Латч команди скиду: параметри фіксуються один раз і не змінюються між
  // повторами, інакше чекер валить перевірку "повтор - та сама команда".
  struct PendingDrop {
    float lat{0.0F};
    float lon{0.0F};
    float alt{0.0F};
    int attempts{0};
    bool acked{false};
    std::chrono::steady_clock::time_point lastSend{};
  };
  std::optional<PendingDrop> pendingDrop{};

  // time_boot_ms має бути монотонним, тому рахуємо його від старту процесу,
  // а не з таймстемпів, що приходять по UART.
  std::chrono::steady_clock::time_point bootTime{std::chrono::steady_clock::now()};
  uint32_t bootMs() const;

public:
  explicit MavLink(std::unique_ptr<comms::SocketLink> link)
    : link(std::move(link))
  {
  }

  void on_message(const mavlink_message_t *m);
  void send_msg(const mavlink_message_t *m);
  // Вичитує всі датаграми, що накопичились у черзі сокета, і парсить їх.
  // Нас цікавлять лише HEARTBEAT (живість лінка) і COMMAND_ACK (підтвердження
  // скиду): позицію та орієнтацію ми віддаємо зі своїх x/y/altitude, а не
  // вичитуємо назад із MAVLink. Повертає кількість розібраних повідомлень.
  int rx_poll();
  void send_heartbeat();
  void send_attitude(const Attitude &a);
  void send_global_position(const GlobalPosition &gp);
  // Зафіксувати координати скиду. Повторний виклик ігнорується - параметри
  // мають лишатись тими самими для всіх спроб.
  void requestDrop(float lat, float lon, float alt);
  // Відправити/повторити команду скиду, якщо ще не було ACK. Викликати регулярно.
  void serviceDrop();
  bool dropAcked() const;
  int dropAttempts() const;

  bool isOpen() const { return this->link && this->link->isOpen(); }
  bool hasPeer() const { return this->link && this->link->hasPeer(); }

  uint64_t receivedMessages() const;
  // Час (ms since epoch) останнього HEARTBEAT від компонента, 0 якщо не було.
  uint64_t lastHeartbeatFrom(uint8_t compid) const;
};
}  // namespace comms
