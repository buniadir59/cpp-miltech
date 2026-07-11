//#include "mission/"
#include "autopilot/autopilot.hpp"
#include "link/drone_link.h"

#include <cstdint>

// Надіслати команду керування:
void sendControl(int fd, float accel, float turnRate) {
Control c{ accel, turnRate }; // обидва float у [-1..1]
uint8_t out[64];
size_t m = encode(PKT_CONTROL, &c, sizeof c, out);
write(fd, out, m);
}