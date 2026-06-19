# C++ для військових технологій: ДЗ#7

## Постановка задачі

   Рефакторинг коду з ДЗ3 і рефакторити його з монолітного main() в набір класів з чіткими інтерфейсами

## **Що змінюється:**

   1. Створені абстрактні класи (інтерфейси) для кожного компонента системи:
      * ITargetProvider: getTargetCount(); getTarget(int idx). Це Провайдер цілей => кількість та дані кожної цілі (позиція, швидкість)
      * IBallisticSolver: solve(dronePos, targetPos, altitude, ammo). Калькулятор балістики => обчислює точку скиду
      * IConfigLoader: load(source); getConfig(); getAmmoParams(). Завантажувач даних => конфіг місії та параметри боєприпасу
      * ISimulationClock: nowS(); nowForTargetProvider(); advance(); reset(double simTimeStep, double tgtTimeStep). Provides 
         source of time for MissionController and JsonTargetProvider
   Кожен інтерфейс — це клас з pure virtual методами (= 0) і virtual деструктором.
   2. Для кожного інтерфейсу написана реалізацію:
      * ITargetProvider => JsonTargetProvider => Завантажує цілі з JSON-файлу
      * BallisticSolver => AnalyticalSolver => Аналітичне рішення (формула з ДЗ1)
      * IConfigLoader => FileConfigLoader => Читає конфіг
      * ManualSimulationClock => provides for adequate moving within the Target simulation data for Json target provider
   3. Додана фабрика, яка створює потрібну реалізацію за типом: createSolver, createProvider, createLoader; createSimulationClock
   4. Створений клас MissionProcessor, який приймає компоненти через вказівники на інтерфейси (патерн Стратегія).
   5. main() виконує: Створення компонентів, їх ініціалізацію, пошагову обробку всіх цілей, видалення створених об'єктів.

## Структура репо

```
homework_07/        
├── external/nlohmann
| | └── json.hpp   
├── include/               ## declarations of classes, interfaces structures 
│ ├── MissionProcessor.hpp
│ ├── Mission.hpp
│ ├── DroneControl.hpp
│ ├── TargetControl.hpp
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
├── src/                   ## implementation of methods
│ ├── main.cpp
│ ├── MissionProcessor.cpp
│ ├── Mission.cpp
│ ├── DroneControl.cpp
│ ├── TargetControl.cpp
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
