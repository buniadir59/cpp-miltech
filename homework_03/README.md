# C++ для військових технологій: ДЗ#3

   Це продовження ДЗ 2 (симуляція дрона). Ви берете свій код з ДЗ 2 та рефакторите його згідно з вимогами нижче. Вся логіка симуляції (автомат станів, балістика, lead targeting, інтерполяція цілей) залишається такою ж.

## **Що змінюється:**

   1. Структури (struct) замість окремих змінних та паралельних масивів - done in scope of HW#2
   2. Перевантаження операторів для координат - done in scope of HW#2
   3. Динамічні масиви (new[]/delete[]) замість хардкоджених розмірів TODO
   4. Формат JSON для всіх файлів введення/виведення (бібліотека nlohmann/json) TODO
   5. #define макроси для виведення в консоль та дебаг-інформації - Done


## Additionally
   1.  TODO : ballistics libruary, which was not part of HW2(taken as ready from HW6) reworked to optimize calculations
 



## Структура репо

```
.homework_03/    
├── data/         # json-files
├── include       # includes for own src & json.hpp
├── src
├── CMakeLists.txt       
└── README.md               
```
