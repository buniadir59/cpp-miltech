# C++ для військових технологій: ДЗ#9

## Strange assumptions :)

   - during simulation run, we can use ammo of one type in unlimited quantities

## Постановка задачі

   Рефакторинг коду з ДЗ8:  


## **Що змінюється:**

   1. Implemented smart pointers - change ownership
   2. Updated ammo parameters
   3. Added new IBallisticSolver implementation class - TableSolver
   4. Implemeted Drone State Automate


## Структура репо

```
homework_09/   
├── CMakeLists.txt       
├── README.md    
├── data/
│ ├── ammo.json
| ├── ballistic_table.txt
│ ├── config.json
│ └── targets.json    
├── external/nlohmann
|  └── json.hpp   
├── include/                  ## declarations of classes, interfaces structures
│ ├── config/
│ | ├── defines.hpp
│ | ├── ManualSimulationClock.hpp
│ | ├── FileConfigLoader.hpp
│ | └── ComponentFactory.hpp
│ ├── core
│ | ├── MissionProcessor.hpp
│ | ├── DroneControl.hpp
│ | └── TargetControl.hpp
│ ├── drone
│ | ├── Acceleratin.hpp
│ | ├── Decelerating.hpp
│ | ├── DroneContext.hpp
│ | ├── Moving.hpp
│ | ├── Stopped.hpp
│ | └── Turning.hpp
│ ├── dto
│ | ├── Ammo.hpp
│ | ├── BallisticsіInput.hpp
│ | ├── DropSolution.hpp
│ | ├── MissionConfig.hpp
│ | ├── SimStatistics.hpp
│ │ └── Target.hpp
│ ├── interfaces/
│ │ ├── ISimulationClock.hpp
│ │ ├── ITargetProvider.hpp
│ │ ├── IBallisticSolver.hpp
│ │ ├── IDroneState.hpp
│ │ ├── IMissionState.hpp
│ │ └── IConfigLoader.hpp
│ ├── math/
│ │ ├── angle_math.hpp
│ │ └── point_math.hpp
│ ├── mission/
│ │ ├── Attack.hpp
│ │ ├── Idle.hpp
│ │ └── MissionCtx.hpp
│ ├── providers/
│ │ └── JsonTargetProvider.hpp
│ └──  solvers/
│   └── AnalyticalSolver.hpp
|   ├── BallisticTable.hpp
|   └── TableSolver.hpp
└── src/                      ## implementation of methods
  ├── main.cpp
  ├── config/
  | ├── ManualSimulationClock.cpp
  | ├── FileConfigLoader.cpp
  | └── ComponentFactory.cpp
  ├── core
  │ ├── MissionProcessor.cpp
  │ ├── DroneControl.cpp
  │ └── TargetControl.cpp
  ├── drone
  | ├── Accelerating.cpp
  | ├── Decelerating.cpp
  | ├── DroneContext.cpp
  | ├── Moving.cpp
  | ├── Stopped.cpp
  | └── Turning.cpp
  ├── math/
  │ ├── angle_math.cpp
  │ └── point_math.cpp
  │ ├── mission/
  │ ├── Attack.cpp
  │ ├── Idle.cpp
  │ └── MissionCtx.cpp
  ├── providers/
  │ └── JsonTargetProvider.cpp
  └── solvers/
    ├── AnalyticalSolver.cpp
    └── TableSolver.cpp
```
## List of TODOs for the future:

   - separate input needed for ballistic table from one for drop route
   - додати можливість скиду на довільній швидкості with correct ballistic data
   - add calculation for decelerate on the move to give more time to turn
   - for point_math: implement near(accuracy) instead of operator==
   - change interface IBallisticSolver to return separately time/distance of ammo fly  and dropRoute
   - TODO use f.good() vs f.fail() in load()
