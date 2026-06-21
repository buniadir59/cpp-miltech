#pragma once

#include "interfaces/IDroneState.hpp"

namespace drone {


class Accelerating final : public  IDroneState {
public:
    std::unique_ptr<IDroneState> execute(drone::DroneContext& ctx);
    const char* name() const { return "ACCELERATING";};
};

}