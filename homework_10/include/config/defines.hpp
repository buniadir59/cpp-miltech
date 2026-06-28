#pragma once

#include <iostream>  //for cout

/* **** defines and contants **** */
// TODO [[maybe_unused]] auto& dummy = std::cout; //"костиль"

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

namespace defines {

inline auto kSimulationPath = "simulation.json";
inline auto kDebugTxtPath = "homework_09/debug_09.txt";

inline auto kBallisticTablePath = "homework_09/data/ballistic_table.txt";
inline auto kConfigPath = "homework_09/data/config.json";
inline auto kAmmoTablePath = "homework_09/data/ammo.json";
inline auto kTargetsPath = "homework_09/data/targets.json";

// ## max number of simulation steps if any target not hit
inline constexpr int kMaxSteps = 10000;

}  // namespace defines