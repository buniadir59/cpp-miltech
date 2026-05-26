# C++ для військових технологій: ДЗ#7

## Постановка задачі

   Рефакторинг коду з ДЗ3 і рефакторити його з монолітного main() в набір класів з чіткими інтерфейсами

## **Що змінюється:**

   1. Створені абстрактні класи (інтерфейси) для кожного компонента системи:
      * ITargetProvider: getTargetCount(); getTarget(int idx). Це Провайдер цілей => кількість та дані кожної цілі (позиція, швидкість)
      * IBallisticSolver: solve(dronePos, targetPos, altitude, ammo). Калькулятор балістики => обчислює точку скиду
      * IConfigLoader: load(source); getConfig(); getAmmoParams(). Завантажувач даних => конфіг місії та параметри боєприпасу
   Кожен інтерфейс — це клас з pure virtual методами (= 0) і virtual деструктором.
   2. Для кожного інтерфейсу написана реалізацію:
      * ITargetProvider => JsonTargetProvider => Завантажує цілі з JSON-файлу
      * BallisticSolver => AnalyticalSolver => Аналітичне рішення (формула з ДЗ1)
      * IConfigLoader => FileConfigLoader => Читає конфіг
   3. Додана фабрика, яка створює потрібну реалізацію за типом: createSolver, createProvider, createLoader; 
   4. Створений клас MissionProcessor, який приймає компоненти через вказівники на інтерфейси (патерн Стратегія).
   5. main() виконує: Створення компонентів, їх ініціалізацію, пошагову обробку всіх цілей, видалення створених об'єктів.

## Структура репо

```
homework_07/            
├── include/               ## declarations of classes, interfaces structures 
│ ├── Types.h
│ ├── MissionProcessor.h
│ ├── interfaces/
│ │ ├── ITargetProvider.h
│ │ ├── IBallisticSolver.h
│ │ └── IConfigLoader.h
│ ├── solvers/
│ │ └── AnalyticalSolver.h
│ ├── providers/
│ │ └── JsonTargetProvider.h
│ └── config/
│   ├── FileConfigLoader.h
│   └── ComponentFactory.h
├── src/                   ## implementation of methods
│ ├── main.cpp
│ ├── MissionProcessor.cpp
│ ├── solvers/
│ │ └── AnalyticalSolver.cpp
│ ├── providers/
│ │ └── JsonTargetProvider.cpp
│ └── config/
│   ├── FileConfigLoader.cpp
│   └── ComponentFactory.cpp
├── data/
│ ├── ammo.json
│ ├── config.json
│ └── targets.json
├── CMakeLists.txt       
└── README.md   
```
