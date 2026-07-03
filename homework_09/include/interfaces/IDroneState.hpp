#pragma once

#include <memory>

namespace drone {
struct DroneContext;
}

class IDroneState {
public:
    virtual ~IDroneState() = default;

    virtual std::unique_ptr<IDroneState> execute(drone::DroneContext& ctx) = 0;
    virtual const char* name() const = 0;
};