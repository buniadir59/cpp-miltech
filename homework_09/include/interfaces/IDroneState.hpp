#pragma once

#include "drone/DroneContext.hpp"

#include <memory>

class IDroneState {
public:
    virtual ~IDroneState() = default;

    virtual std::unique_ptr<IDroneState> execute(drone::DroneContext& ctx) = 0;
    virtual const char* name() const = 0;
};