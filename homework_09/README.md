# C++ для військових технологій: ДЗ#9

## Постановка задачі

   Рефакторинг коду з ДЗ8:  


## **Що змінюється:**

   1.
   2.


## Структура репо

```
homework_09/        
├── external/nlohmann
|  └── json.hpp   
├── include/                  ## declarations of classes, interfaces structures 
│ ├── core
│ | ├── MissionProcessor.hpp
│ | ├── Mission.hpp
│ | ├── DroneControl.hpp
│ | └── TargetControl.hpp
│ ├── math/
│ │ ├── angle_math.hpp
│ │ └── point_math.hpp
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
│ │ └── IConfigLoader.hpp
│ ├── solvers/
│ │ └── AnalyticalSolver.hpp
│ ├── providers/
│ │ └── JsonTargetProvider.hpp
│ └── config/
│   ├── defines.hpp
│   ├── ManualSimulationClock.hpp
│   ├── FileConfigLoader.hpp
│   └── ComponentFactory.hpp
├── src/                      ## implementation of methods
│ ├── main.cpp
│ ├── core
│ │ ├── MissionProcessor.cpp
│ │ ├── Mission.cpp
│ │ ├── DroneControl.cpp
│ │ └── TargetControl.cpp
│ ├── math/
│ │ ├── angle_math.cpp
│ │ └── point_math.cpp
│ ├── solvers/
│ │ └── AnalyticalSolver.cpp
│ ├── providers/
│ │ └── JsonTargetProvider.cpp
│ └── config/
│   ├── ManualSimulationClock.cpp
│   ├── FileConfigLoader.cpp
│   └── ComponentFactory.cpp
├── data/
│ ├── ammo.json
│ ├── config.json
│ └── targets.json
├── CMakeLists.txt       
└── README.md   
```
