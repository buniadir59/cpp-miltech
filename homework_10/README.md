# C++ для військових технологій: ДЗ#10

## Постановка задачі

    Багатопоточність — цілі, фізика та місія в окремих потоках.
    Мета: навчитися працювати з std::thread, std::mutex, std::atomic та потокобезпечними структурами даних.
    Важливо: симуляція тепер іде в реальному часі — потоки сплять між кроками. Результат перестає бути детермінованим: 
    два запуски дадуть трохи різні траєкторії. Це нормально.

## **Що змінюється:**

3 threads / 3 classes:
1. ThreadSafeTargetProvider рухає цілі вздовж траєкторій, віддає поточні позицію та швидкість. Period - targetTimeStep
2. DronePhysics виконує команди, інтегрує позицію, швидкість і напрямок дрона. Period - physicsTimeStep
3. MissionProcessor обирає ціль, рахує балістику, керує станами, командує фізикою. Period - simTimeStep

Хто чим володіє:
•       Провайдер володіє позиціями та швидкостями цілей. Назовні віддає лише копії поточних значень.
•       Фізика володіє станом дрона: позицією, швидкістю, напрямком. MissionProcessor власної копії стану дрона не зберігає 
— щоразу запитує фізику.
•       MissionProcessor володіє логікою місії: вибір цілі, балістика, стейт-машина.

Дані між потоками рухаються лише як копія-знімок під м'ютексом (телеметрія дрона, цілі) та (команди фізиці). Прямий доступ 
до полів іншого потоку заборонено.

##  Вимоги
•       Усі спільні дані — під std::mutex або std::atomic. Жодного доступу до полів іншого потоку без синхронізації.
•       Стоп-прапорці — std::atomic<bool>; потоки завершуються через join(), без detach().
•       Черга команд — власний шаблонний клас ThreadSafeQueue.
•       Цілі починають рухатися лише за сигналом start() — після того, як isThreadReady() підтвердив готовність потоку.
•       MissionProcessor не зберігає та не інтегрує стан дрона.
•       Target містить лише поточні позицію та швидкість — без масиву координат.

## Підказки
•       Порядок розробки: спочатку зробіть DronePhysics і ThreadSafeQueue без потоків — викликайте крок фізики вручну
 з циклу місії та переконайтеся, що симуляція працює як у ДЗ9. Потім розносьте по потоках.
•       #include <thread>, <mutex>, <atomic>, <chrono>, <queue>
•       Сон на дробову кількість секунд: std::this_thread::sleep_for(std::chrono::duration<float>(dt / timeScale))
•       Тримайте критичні секції короткими: під lock_guard — лише копіювання даних. Жодних обчислень і тим більше sleep під замком.
•       mutable std::mutex — щоб блокувати м'ютекс у const-методах (getTelemetry, getTarget).
•       physicsTimeStep має бути меншим за simTimeStep: фізика повинна встигати виконати команди між кроками планувальника.

•       Під Linux/WSL зберіть з -fsanitize=thread — ThreadSanitizer покаже гонки даних, які неозброєним оком не видно

## Структура репо

```
homework_09/         //TODO
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

## Strange assumptions :)

   - during simulation run, we can use ammo of one type in unlimited quantities

## List of TODOs for the future:

   - for point_math: implement near(accuracy) instead of operator==

   
 
