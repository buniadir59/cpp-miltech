#pragma once

#include <memory>

#include "interfaces/IMissionState.hpp"

namespace mission {


class Attack final : public IMissionState {
public:
    std::unique_ptr<IMissionState> execute(MissionCtx& ctx) override;
    const char* name() const override { return "M_ATTACK";};
};

}