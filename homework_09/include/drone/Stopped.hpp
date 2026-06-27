#pragma once

#include <memory>

#include "interfaces/IDroneState.hpp"

namespace drone {


class Stopped final : public IDroneState {
public:
    std::unique_ptr<IDroneState> execute(DroneContext& ctx) override;
    const char* name() const override { return "STOPPED";};
};

}