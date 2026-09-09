#pragma once

#include <netinet/in.h>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>

namespace comms {

// UDP-ендпоінт для MAVLink.
//
// Сокет біндиться на 0.0.0.0:port і працює у двох режимах адресації:
//   * якщо задано remoteHost/remotePort - шлемо на цю фіксовану адресу;
//   * інакше адреса піра вивчається з першої отриманої датаграми (класична
//     схема "GCS сам знаходить дрон"), і далі відповіді йдуть туди.
// Поки пір невідомий, send() нічого не робить і повертає 0 - це краще, ніж
// слати на 0.0.0.0 і отримувати власні пакети назад через loopback.
class SocketLink {
private:
  int socket_fd{-1};
  int port{0};
  sockaddr_in addr{};  // локальна адреса (bind)

  // peer_addr пишеться потоком опитування (receive) і читається потоком
  // heartbeat (send), тому доступ до нього під м'ютексом.
  mutable std::mutex peer_mtx;
  sockaddr_in peer_addr{};  // куди слати: задано явно або вивчено з rx
  std::atomic<bool> peer_known{false};

public:
  explicit SocketLink(int port, const std::string &remoteHost = {}, int remotePort = 0);
  ~SocketLink();

  SocketLink(const SocketLink &) = delete;
  SocketLink &operator=(const SocketLink &) = delete;

  bool isOpen() const { return this->socket_fd >= 0; }
  bool hasPeer() const { return this->peer_known.load(); }
  std::string peerDescription() const;

  // Повертає кількість відправлених байтів, 0 якщо пір ще невідомий, -1 при помилці.
  int send(const uint8_t *buf, size_t len);
  // Неблокуюче читання однієї датаграми. -1 і errno==EAGAIN означає "більше нічого".
  int receive(uint8_t *buf, size_t len);
};
}  // namespace comms
