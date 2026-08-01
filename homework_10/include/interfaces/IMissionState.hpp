#pragma once

#include <memory>

namespace mission {
struct MissionCtx;
}

class IMissionState {
public:
    virtual ~IMissionState() = default;

    virtual std::unique_ptr<IMissionState> execute(mission::MissionCtx& ctx) = 0;
    virtual const char* name() const = 0;
};