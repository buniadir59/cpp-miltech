#include "c2_controller.hpp"
#include "fc_link.hpp"     // MAVSDK обгортка, API описано у fc_link.hpp
#include "udp_socket.hpp"  // UDP прийом, API описано у udp_socket.hpp

#include <nlohmann/json.hpp>  // Розбiр JSON з точками маршруту вiд auto_stub

#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <sstream>
#include <cstddef>

/*
read FC state
read waypoint
apply safety policy - forward or block

| C2 state       | Armed | FC mode   | Waypoint  | Додаткова дія               |
| -------------- | ----: | --------- | --------- | --------------------------- |
| `DISARMED`     |    no | будь-який | blocked   | нічого                      |
| `ARMED_HOLD`   |   yes | Hold      | blocked   | `hold()` один раз при вході |
| `ARMED_GUIDED` |   yes | Guided    | forwarded | `go_to_ned()`               |
| `ARMED_MANUAL` |   yes | Manual    | blocked   | не заважати оператору       |
*/

namespace {

constexpr std::size_t kUdpMaxSize = 1500;  // розмір буфера для читання пакета

struct Waypoint {
  float north_m{0.0};
  float east_m{0.0};
  bool has_data = false;
};

Waypoint parseWaypointPacket(const std::array<char,kUdpMaxSize>& buffer, std::size_t count)
{
  Waypoint result{};

  try {
    const auto data = nlohmann::json::parse(buffer.begin(), buffer.begin() + count);

    result.north_m = data.at("north_m").get<float>();
    result.east_m = data.at("east_m").get<float>();
    result.has_data = true;
  }
  catch (const nlohmann::json::exception& e) {
    std::cerr << "[C2] error: invalid waypoint: " << e.what() << '\n';
    // signal_unhealthy() - not required
  }

  return result;
}

std::string stateToString(C2State state)
{
  switch (state) {
    case C2State::DISARMED:
      return "DISARMED";
    case C2State::ARMED_HOLD:
      return "ARMED_HOLD";
    case C2State::ARMED_GUIDED:
      return "ARMED_GUIDED";
    case C2State::ARMED_MANUAL:
      return "ARMED_MANUAL";
    default:
      return "Unknown_state";
  }
}

}  // namespace

static constexpr uint16_t STUB_PORT = 14560;

struct C2Controller::Impl {
  FcLink fc;           // блокується до вiдповiдi автопiлота (30 s)
  std::ofstream flog; 
  C2State state = C2State::DISARMED;
  UdpSocket udp_stub;
  bool health_signalled = false;
  std::array<char, kUdpMaxSize> buffer{};

  // constructor приймає fc_port і створює fc object, також  створює об'єкти
  // udp-сокета і лог-файла та відкриває лог-файл "/var/log/c2/c2.log"
  explicit Impl(uint16_t fc_port)
    : fc(fc_port)
    , flog("/var/log/c2/c2.log")
    , udp_stub(STUB_PORT)  // UdpSocket має слухати STUB_PORT.
  {
    if (!flog.is_open()) {
      throw std::runtime_error("[C2] error: cannot open /var/log/c2/c2.log\n");
    }
  }

  void update_connection_health()
  {
    if (!health_signalled && fc.is_connected()) {  // <= got first HEARTBEAT from fc
      std::ofstream("/tmp/c2_healthy").close();
      health_signalled = true;
    }
  }

  // якщо next != state, записати у stdout i лог: [C2] state: PREV -> NEW, потiм оновити state.
  void transition(C2State next)
  {
    if (next != state) {
      // для входу в ARMED_HOLD має бути організований одноразовий hold()
      if (next == C2State::ARMED_HOLD) {
        fc.hold();
      }
      std::string s = "[C2] state: " + stateToString(state) + " -> " + stateToString(next) + '\n';
      log(s);
      state = next;
    }
  }

  void update_state()
  {
    C2State next = state;
    // Визначити актуальний стан
    if (!fc.is_armed()) {
      next = C2State::DISARMED;
    }
    else {
      switch (fc.flight_mode()) {
        case FcLink::FlightMode::Hold:
          next = C2State::ARMED_HOLD;
          break;
        case FcLink::FlightMode::Guided:
          next = C2State::ARMED_GUIDED;
          break;
        case FcLink::FlightMode::Manual:
        default:
          next = C2State::ARMED_MANUAL;
          break;
      }
    }
    // Зробити перехід
    transition(next);
  }

  void process_waypoint()
  {
    // try to read waypoint w/UdpSocket unblocking => recv(...) might return -1
    // if no packet
    const auto bytes_received = udp_stub.recv(buffer.data(), buffer.size());

    if (bytes_received > 0) {
      // parse JSON, check status and forward or block
      Waypoint wpt = parseWaypointPacket(buffer, bytes_received);
      if (wpt.has_data) {
        std::string s;
        if (state == C2State::ARMED_GUIDED) {
          fc.go_to_ned(wpt.north_m, wpt.east_m);
          std::ostringstream oss;
          oss << "[C2] fwd: north=" << wpt.north_m << " east=" << wpt.east_m << '\n';
          s = oss.str();
        }
        else {
          s = "[C2] blocked: waypoint in " + stateToString(state) + '\n';
        }
        log(s);
      }
    }
  }

  void log(const std::string& message)
  {
    std::cout << message << std::flush;
    flog << message << std::flush;
  }

};  // eo Impl struct

// transfer fc_port to and call Impl constructor
C2Controller::C2Controller(uint16_t fc_port)
  : impl_(std::make_unique<Impl>(fc_port))
{
}

C2Controller::~C2Controller() = default;

void C2Controller::tick()
{
  // healthcheck
  impl_->update_connection_health();

  // update C2State
  impl_->update_state();

  // read waypoint, and process it according to the current state
  impl_->process_waypoint();
}

C2State C2Controller::current_state() const
{
  return impl_->state;
}
