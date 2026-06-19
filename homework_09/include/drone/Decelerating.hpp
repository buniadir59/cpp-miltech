#pragma once

#include "interfaces/IDroneState.hpp"

namespace drone {


class Decelerating final :  IDroneState {

    std::unique_ptr<IDroneState> execute(drone::DroneContext& ctx);
    const char* name() const { return "DECELERATING";};
};

}