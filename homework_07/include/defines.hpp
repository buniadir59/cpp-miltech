#pragma once

#include <limits>
/* **** defines and contants **** */

#define ENABLE_LOG 1
#define ENABLE_DEBUG 0

#if ENABLE_LOG
#define LOG(msg) std::cout << "[LOG] " << msg << std::endl;
#else
#define LOG(msg)
#endif

#if ENABLE_DEBUG
#define DEBUG(msg) std::cout << "[DEBUG] " << msg << std::endl;
#else
#define DEBUG(msg)
#endif

#define START_FROM_TGT 3
#define DEBUG_FILE_NAME "debug_07.txt"
//#define TESTOUT_TO_FILE

namespace  defines {


const char* const kInputPath = "homework_07/data";
const char* const kSimulationPath = "simulation.json";

// ## max number of simulation steps if any target not hit
constexpr int kMaxSteps = 10000; //TODO !!! 10000;

constexpr int kMaxTargets = 32; //max number of targets
constexpr int kMaxRecalculations = 6; //for drop route

constexpr double eps = std::numeric_limits<double>::epsilon();

} 