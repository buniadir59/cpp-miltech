#pragma once

#include <limits>
#include <string>

/* **** defines and contants **** */

// #define TESTOUT_TO_FILE
#define ENABLE_LOG 1
#define ENABLE_DEBUG 0

#if ENABLE_LOG
#define LOG(msg) std::cout << "[LOG] " << msg << '\n';
#else
#define LOG(msg)
#endif

#if ENABLE_DEBUG
#define DEBUG(msg) std::cout << "[DEBUG] " << msg << '\n';
#else
#define DEBUG(msg)
#endif

#define DEBUG_FILE_NAME "debug_08.txt"

namespace defines {

const std::string kInputPath = "homework_08/data";
const char* const kSimulationPath = "simulation.json";

// ## max number of simulation steps if any target not hit
constexpr int kMaxSteps = 10000;

constexpr int kMaxTargets = 32;        // max number of targets
constexpr int kMaxRecalculations = 6;  // for drop route

constexpr double eps = std::numeric_limits<double>::epsilon();

}  // namespace defines