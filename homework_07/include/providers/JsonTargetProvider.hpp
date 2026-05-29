#pragma once

#include "dto/Target.hpp"
#include "interfaces/ITargetProvider.hpp"
#include "interfaces/ISimulationClock.hpp"
#include "math/point_math.hpp"

// Завантажує таблицю координат цілей з JSON-файлу, 
// повертає ціль(координати і швидкість) згідно з поточним часом симуляції
class JsonTargetProvider final : public ITargetProvider {
    const char* const kTgtsFileName = "targets.json";

    static constexpr size_t kMaxTargetCount = 64; 
    static constexpr size_t kMaxTargetTimeSteps = 200;
    
    size_t tgtCount = 0;        //number of active targets  
    double tgtTimeStep = 0.0;     //time between tgt steps in the pos array
    size_t nOfTgtTimeSteps = 0; //length of pos array
    pointmath::Point** tgtTracks = nullptr; //array of coordinates
    const ISimulationClock* clock{nullptr};

    auto parseJson(const char* source) -> void;
    auto makeTarget(size_t timeIdx, const pointmath::Point* track) -> dto::Target;

public:
    JsonTargetProvider(const char* path, double tgtTimeStep, const ISimulationClock* clock) 
        : tgtTimeStep(tgtTimeStep), clock(clock) 
    {       
        parseJson(path);
    }
    
    ~JsonTargetProvider();

    auto getTargetCount() -> int override { return tgtCount; }

    auto getTarget(int idx) -> dto::Target override;
};

