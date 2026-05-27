#pragma once

#include "dto/Target.hpp"

class ITargetProvider {

public:
    virtual int getTargetCount() = 0;
    virtual dto::Target getTarget(int index) = 0;
    virtual ~ITargetProvider() {}
};