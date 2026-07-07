# C++ для військових технологій: ДЗ#10

## Постановка задачі

    Багатопоточність — цілі, фізика та місія в окремих потоках.
    Мета: навчитися працювати з std::thread, std::mutex, std::atomic та потокобезпечними структурами даних.
    Важливо: симуляція тепер іде в реальному часі — потоки сплять між кроками. Результат перестає бути детермінованим: 
    два запуски дадуть трохи різні траєкторії. Це нормально.

## **Що змінюється:**
Зміни щодо зауважень з рев'ю  ДЗ-9:
1) прибрані залишки невикористовуваного кода щодо планування маршруту з IBallisticSolver і його імплементацій, що тягнулися з ДЗ-3.
2) момент скиду позначений спеціальним станом в вихідному json-файлі (FIRED=5).    
3) доданий аналіз командного рядка - якщо в ньому є шляхи до вхідних файдів конфігурації, ammo, симіляції цілей, 
  балістичної таблиці і вихідного файла симуляції. Якщо значення відсутні, то використовуються значення з defines.hpp

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

## Структура репо

```
homework_10/       
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
│ | ├── FileConfigLoader.hpp
│ | └── ComponentFactory.hpp
│ ├── core
│ | ├── MissionProcessor.hpp
│ | ├── ThreadSafeQueue.hpp
│ | ├── TimeTracker.hpp
│ | └── TargetControl.hpp
│ ├── drone
│ | └── DronePhysics.hpp
│ ├── dto
│ | ├── Ammo.hpp
│ | ├── BallisticsInput.hpp
│ | ├── BallisticResult.hpp
│ | ├── DroneInterfaceStructures.hpp
│ | ├── MissionConfig.hpp
│ | ├── SimStatistics.hpp
│ │ └── Target.hpp
│ ├── interfaces/
│ │ ├── ITargetProvider.hpp
│ │ ├── IBallisticSolver.hpp
│ │ ├── IDroneState.hpp
│ │ ├── IMissionState.hpp
│ │ └── IConfigLoader.hpp
│ ├── math/
│ │ ├── angle_math.hpp
│ │ └── point_math.hpp
│ ├── mission/
│ │ └── MissionCtx.hpp
│ ├── providers/
│ │ └── ThreadSafeTargetProvider.hpp
│ └──  solvers/
│   ├── AnalyticalSolver.hpp
|   ├── BallisticTable.hpp
|   └── TableSolver.hpp
└── src/                      ## implementation of methods
  ├── main.cpp
  ├── config/
  | ├── FileConfigLoader.cpp
  | └── ComponentFactory.cpp
  ├── core
  │ ├── MissionProcessor.cpp
  │ └── TargetControl.cpp
  ├── drone
  | └── DronePhysics.cpp
  ├── math/
  │ ├── angle_math.cpp
  │ └── point_math.cpp
  ├── mission/
  │ └── MissionCtx.cpp
  ├── providers/
  │ └── ThreadSafeTargetProvider.cpp
  └── solvers/
    ├── AnalyticalSolver.cpp
    └── TableSolver.cpp
```

## Strange assumptions :)

   - during simulation run, we can use ammo of one type in unlimited quantities

## List of TODOs for the future:

   - for point_math: implement near(accuracy) instead of operator==

   
 
