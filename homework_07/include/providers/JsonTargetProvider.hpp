#pragma once

#include "dto/Target.hpp"
#include "interfaces/ITargetProvider.hpp"


class JsonTargetProvider : public ITargetProvider {
    
    dto::Target targets[64];
    int count;

    int parseJson(const char* source, dto::Target&, const int number_of_targets);

public:
    JsonTargetProvider(const char* path) {
        count = parseJson(path, targets, 64);
    }

    int getTargetCount() override { return count; }

    dto::Target getTarget(int i) override { return targets[i]; }
};