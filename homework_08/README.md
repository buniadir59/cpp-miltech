# C++ для військових технологій: ДЗ#7

## Постановка задачі

   Рефакторинг коду з ДЗ7:  Правильно структурувати проект та замінити C-стиль конструкції на STL-контейнери всюди, де це має сенс. 


## **Що змінюється:**

   1. Структура проекту загалом була створена в ДЗ-7 і приведена нижче
   2. Виконана заміна на STL-контейнери
      * array of targets under control (targetDepo) is replaced by vector 
      * array tgtTracks replaced by vector
      * array of Ammos replaced by vector
      * standard algorithm count_if() was used to create simulation statictics 


## Структура репо

```
homework_07/        
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
│ | ├── BallisticInput.hpp
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
