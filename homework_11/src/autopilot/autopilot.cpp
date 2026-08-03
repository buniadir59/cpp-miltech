// #include "mission/"
#include "autopilot/autopilot.hpp"
#include "link/drone_link.h"

#include <cstdint>

namespace autopilot {

using Control = dlink::Control;

void write (int fd, const uint8_t* data, size_t size) { //TODO

}

// Надіслати команду керування:
void sendControl(int fd, float accel, float turnRate)
{
  Control c{accel, turnRate};  // обидва float у [-1..1]
  uint8_t out[64];
  size_t m = dlink::encode(dlink::PKT_CONTROL, &c, sizeof c, out);
  write(fd, out, m);
}

}  // namespace autopilot