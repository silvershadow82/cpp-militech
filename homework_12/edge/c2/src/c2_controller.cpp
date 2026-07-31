#include "c2_controller.hpp"
#include "fc_link.hpp"  // MAVSDK обгортка, API описано у fc_link.hpp
#include "nlohmann/json_fwd.hpp"
#include "udp_socket.hpp"  // UDP прийом, API описано у udp_socket.hpp

#include <cstdio>
#include <filesystem>
#include <nlohmann/json.hpp>  // Розбiр JSON з точками маршруту вiд auto_stub

#include <fstream>
#include <iostream>

static constexpr uint16_t STUB_PORT = 14560;
static constexpr const char* LOG_FILE = "/var/log/c2/c2.log";
static constexpr const char* HEALTHCHECK_FILE = "/tmp/c2_healthy";

struct C2Controller::Impl {
  C2State state = C2State::DISARMED;
  FcLink fcLink;
  UdpSocket socket;
  std::ofstream log;

  Impl(uint16_t fc_port)
    : fcLink(fc_port)
    , socket(STUB_PORT)
  {
  }

  void transition(C2State next)
  {
    if (state != next) {
      char buf[32];
      sprintf(buf, "%d -> %d", static_cast<int>(state), static_cast<int>(next));
      std::cout << buf << std::endl;
      log << buf << std::endl;

      state = next;
    }
  }
};

C2Controller::C2Controller(uint16_t fc_port)
  : impl_(std::make_unique<Impl>(fc_port))
{
  impl_->log.open(LOG_FILE);
  if (!impl_->log.is_open()) {
    return;
  }
}

C2Controller::~C2Controller()
{
  if (impl_->log.is_open()) {
    impl_->log.close();
  }
  if (std::filesystem::exists(HEALTHCHECK_FILE)) {
    std::filesystem::remove(HEALTHCHECK_FILE);
  }
};

void C2Controller::tick()
{
  if (!impl_->fcLink.is_connected()) {
    std::cerr << "FC link is disconnected, skipping the tick..." << std::endl;
    return;
  }

  if (!std::filesystem::exists(HEALTHCHECK_FILE)) {
    std::ofstream healthCheck(HEALTHCHECK_FILE, std::ios::out);
  }

  if (!impl_->fcLink.is_armed()) {
    impl_->transition(C2State::DISARMED);
  }
  else {
    switch (impl_->fcLink.flight_mode()) {
      case FcLink::FlightMode::Hold:
        impl_->transition(C2State::ARMED_HOLD);
        impl_->fcLink.hold();
        impl_->log << "[C2] blocked: waypoint in " << static_cast<int>(impl_->state);
        break;
      case FcLink::FlightMode::Guided: {
        impl_->transition(C2State::ARMED_GUIDED);

        char buf[256] = {};
        ssize_t n = impl_->socket.recv(buf, sizeof(buf) - 1);
        if (n <= 0) {
          break;
        }

        auto msg = nlohmann::json::parse(buf, buf + n, nullptr, false);
        if (msg.is_discarded() || !msg.contains("north_m") || !msg.contains("east_m")) {
          impl_->log << "[C2] warn: bad waypoint json" << std::endl;
          break;
        }
        float north_m = msg["north_m"].get<float>();
        float east_m = msg["east_m"].get<float>();

        // [C2] fwd: north=X east=Y
        impl_->log << "[C2] fwd: north=" << north_m << " east=" << east_m << std::endl;

        impl_->fcLink.go_to_ned(north_m, east_m);
        break;
      }
      case FcLink::FlightMode::Manual:
        impl_->transition(C2State::ARMED_MANUAL);
        break;
      case FcLink::FlightMode::Unknown:
        impl_->log << "[C2] blocked: waypoint in " << static_cast<int>(impl_->state);
        break;
      default:
        break;
    }
  }
}

C2State C2Controller::current_state() const
{
  return impl_->state;
}
