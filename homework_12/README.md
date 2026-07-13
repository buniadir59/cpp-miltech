# C++ для військових технологій: ДЗ#12

## Постановка задачі
Є середовище симуляції керування НРК, що складається з чотирьох компонентів:

1 - Симулятор польотного контролера НРК на базі ArduRover SITL — готовий Docker-образ, 
який запускається через окремий Compose-файл sim/compose.sitl.yml — Compose A.
2 - QGroundControl (QGC) — наземна станція керування, через яку оператор вручну 
виконує Arm/Disarm, перемикає режими Guided, Manual, Hold і спостерігає за 
станом НРК. QGC комунікує з FC через UDP-порт 14550.
3 - auto_stub — готовий Docker-образ зі спрощеною автономною логікою. Він читає 
маршрут, генерує повідомлення з waypoint в форматі json ( приклад: 
{"north_m": 50.0, "east_m": 0.0} ) і передає їх через UDP-порт 14560 до сервісу C2.
4 - c2_service — сервіс бортового комп’ютера, який виконує роль safety gate між 
автономною логікою auto_stub і польотним контролером FC. Він визначає, чи можна передати 
отриману waypoint-команду до FC, виконує необхідні дії при зміні стану та записує події 
в лог. c2_service працює разом з auto_stub у Compose B — edge/docker-compose.yml

Задача — реалізувати C2Controller як машину станів.

Стан C2 визначається на основі даних, отриманих від FC:
 * armed/disarmed;
 * поточний режим руху: Guided, Manual або Hold.
Машина станів має чотири стани:
   DISARMED / ARMED_HOLD / ARMED_GUIDED / ARMED_MANUAL.

Правила роботи:
* FC не armed => C2 переходить у DISARMED, waypoint-команди блокуються.
* FC armed + Guided => C2 переходить у ARMED_GUIDED і передає waypoint-команди до FC
 через fc.go_to_ned(north, east).
* FC armed + Hold=> C2 переходить у ARMED_HOLD, один раз при вході в цей стан 
викликає fc.hold() і блокує waypoint-команди.
* FC armed + Manual=> C2 переходить у ARMED_MANUAL, блокує waypoint-команди та 
не втручається в ручне керування.

 C2 записує в лог :
 * [C2] state: PREV -> NEW             При кожній реальній зміні стану
 * [C2] fwd: north=X east=Y            При передачі waypoint
 * [C2] blocked: waypoint in STATE     При блокуванні
 
Лог-файл: /var/log/c2/c2.log
На машинi, де запускається Docker, цей файл має бути доступний як: edge/logs/c2.log

Вихідний код для подальшої роботи наданий в проекті на гітхабі (папка homework_12)

## **Що змінюється:**

   1. edge/ docker-compose.yml  - заповнено пропуски
   2. с2/   Dockerfile          - заповнено пропуски
   3. c2/scr/ c2_controller.cpp - реалізована машини станів згідно ТЗ

Інші компоненти визначають середовище і інтерфейси,вони надані в готовому вигляді 
і не мають бути змінені

## Запуск і перевірка

Для готових контейнерів.

### 1. Запустити ArduRover SITL

У WSL з каталогу `homework_12`:
   GCS_HOST=$(ip route show default | awk '{print $3}') \
   docker compose -f sim/compose.sitl.yml up -d

GCS_HOST задає IP Windows-хоста, щоб QGroundControl, запущений у Windows, отримував MAVLink-пакети з WSL через UDP-порт 14550.

Перевірити стан SITL:
   docker compose -f sim/compose.sitl.yml ps
   docker exec fc_sim sh -lc 'tail -n 30 /tmp/Rover.log'

У логах мають бути UDP-з’єднання для QGC на порт 14550 і для c2_service на порт 14551.

### 2. Запустити edge-стек
В WSL-терміналі:
   cd edge
   docker compose up -d
   docker compose ps

Очікуваний стан:
* auto_stub    healthy
* c2_service   healthy

### 3. Перевірити роботу C2

Запустити QGroundControl у Windows. Після автоматичного підключення до SITL перейти у Fly View і 
вручну перевірити переходи:

* Disarmed;
* Armed + Guided;
* Armed + Hold;
* Armed + Manual;
* повернення з Manual у Guided.

Для C2-сценарію не натискати Start Mission: маршрут передається через  auto_stub -> c2_service -> FC.

Логи C2:
   docker compose logs -f c2_service

Лог-файл на host:
   cat ./logs/c2.log

### 4. Зупинка

Edge-стек:
   docker compose down

SITL:
   cd ..
   docker compose -f sim/compose.sitl.yml down

