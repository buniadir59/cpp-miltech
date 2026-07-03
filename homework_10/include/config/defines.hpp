#pragma once

#include <iostream>  //for cout

/* **** defines and contants **** */
// TODO [[maybe_unused]] auto& dummy = std::cout; //"костиль"

// #define TESTOUT_TO_FILE
#define ENABLE_LOG 1
#define ENABLE_DEBUG 1

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

inline auto kAppname = "homework_10";
inline auto kSimulationPath = "simulation.json";
inline auto kDebugTxtPath = "homework_10/debug_10.txt";

inline constexpr auto kBallisticTable_Path = "homework_10/data/ballistic_table.txt";
inline constexpr auto kConfig_Path = "homework_10/data/config.json";
inline constexpr auto kAmmoTable_Path = "homework_10/data/ammo.json";
inline constexpr auto kTargets_Path = "homework_10/data/targets.json";

// ## max number of simulation steps if any target not hit
inline constexpr int kMaxSteps = 200; //TODO 10000;

}  // namespace defines