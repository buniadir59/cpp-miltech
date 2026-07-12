#include "c2_controller.hpp"
#include "fc_link.hpp"     // MAVSDK обгортка, API описано у fc_link.hpp
#include "udp_socket.hpp"  // UDP прийом, API описано у udp_socket.hpp

#include <nlohmann/json.hpp>  // Розбiр JSON з точками маршруту вiд auto_stub

#include <fstream>
#include <iostream>
#include <string>

/*
read FC state
read waypoint
apply safety policy
forward or block

| C2 state       | Armed | FC mode   | Waypoint  | Додаткова дія               |
| -------------- | ----: | --------- | --------- | --------------------------- |
| `DISARMED`     |    no | будь-який | blocked   | нічого                      |
| `ARMED_HOLD`   |   yes | Hold      | blocked   | `hold()` один раз при вході |
| `ARMED_GUIDED` |   yes | Guided    | forwarded | `go_to_ned()`               |
| `ARMED_MANUAL` |   yes | Manual    | blocked   | не заважати оператору       |

*/
namespace {
std::string stateToString(C2State state)
{
  switch (state) {
    case C2State::DISARMED:
      return "DISARMED";
    case C2State::ARMED_HOLD:
      return "";
    case C2State::ARMED_GUIDED:
      return "";
    case C2State::ARMED_MANUAL:
      return "";
    default:
      return "Unknown_state";
  }
}

}  // namespace

static constexpr uint16_t STUB_PORT = 14560;

struct C2Controller::Impl {
  FcLink fc;           // блокується до вiдповiдi автопiлота (30 s)
  std::ofstream flog;  //("/etc/c2/c2_config.json");
  C2State state = C2State::DISARMED;
  UdpSocket udp_stub;
  // TODO:та прапорцi стану.????

  // якщо next != state, записати "PREV -> NEW" у stdout i лог,
  //  потiм оновити state. Якщо стан не змiнився, нiчого не писати.
  void transition(C2State next)
  {
    if (next != state) {
      std::string s = "[log]" + stateToString(state) + " -> " + stateToString(next) + '\n';
      std::cout << s;
      flog << s;
      state = next;
    }
  }

  // constructor приймає fc_port і створює fc object, також  створює об'єкти
  // udp-сокета і лог-файла та відкриває лог-файл "/var/log/c2/c2.log"
  explicit Impl(uint16_t fc_port)
    : fc(fc_port)
    , flog("/var/log/c2/c2.log")
    , udp_stub(STUB_PORT)  // UdpSocket має слухати STUB_PORT.
  {
    if (!flog.is_open()) {
      std::cerr << "[C2] error: cannot open /var/log/c2/c2.log\n";
    }
  }
};

// передати fc_port в Impl та вiдкрити /var/log/c2/c2.log.
C2Controller::C2Controller(uint16_t fc_port)
  : impl_(std::make_unique<Impl>(fc_port))
{
}

C2Controller::~C2Controller() = default;

/*         try {
            f >> cfg;
        } catch (const nlohmann::json::exception& e) {
            std::cerr << "[C2] error: invalid config: " << e.what() << "\n";

        } */
void C2Controller::tick()
{
  // TODO: healthcheck, 
  // TODO оновлення C2State, 
  // TODO читання точки маршруту,
  // TODO передавання або блокування команди згiдно з поточним станом.

  /*
Кожен tick() має зробити приблизно чотири речі.

1. Перевірити health
if (fc.is_connected()) {
    create /tmp/c2_healthy;
}
2. Визначити актуальний стан
if (!fc.is_armed()) {
    next = DISARMED;
} else {
    switch (fc.flight_mode()) {
        case Hold:
            next = ARMED_HOLD;
            break;
        case Guided:
            next = ARMED_GUIDED;
            break;
        case Manual:
        default:
            next = ARMED_MANUAL;
            break;
    }
}
3. Зробити transition
transition(next);

transition():

нічого не робить, якщо стан не змінився;
логгує PREV -> NEW;
оновлює state;
для входу в ARMED_HOLD має бути організований одноразовий hold().
4. Спробувати прочитати waypoint

UdpSocket неблокуючий.

Тобто:

recv(...)

може повернути:

-1

якщо зараз повідомлення немає.

Тоді tick() просто завершується.

Якщо повідомлення є:

парсить JSON;
дивиться на поточний стан;
або forwarding;
або blocked.
*/

}

C2State C2Controller::current_state() const
{
  return impl_->state;
}
