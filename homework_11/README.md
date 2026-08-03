# C++ для військових технологій: ДЗ#11

## Постановка задачі

Ваша програма — це бортовий автопілот дрона. Чекер симулює політ: 
    шле вам по UART телеметрію (позиція, швидкість, курс, час) і позиції цілей. 
Ви у відповідь шлете по UART команди керування (прискорення і швидкість повороту), якими ведете дрон до цілі, 
    і в потрібний момент даєте імпульс скиду на GPIO. Чекер ловить його, рахує балістику падіння і каже: влучили чи ні.
Замкнений контур: чекер рухає дрон лише за вашими командами. Не шлете керування — дрон нікуди не доведений.
    Тобто треба і кермувати, і вчасно скинути.
Мета: двосторонній обмін по UART (читати телеметрію + слати керування), рішення в реальному часі та керування 
  виходом GPIO — той самий код на платі й у симуляції.

## **Що змінюється:**
Зміни щодо зауважень з рев'ю  ДЗ-10: Done

1. Відкрити і читати UART
Порт — це файл. Відкрити його, налаштувати режим (115200, 8N1, raw) і читати потік байтів. 
Готовий drone_link.h містить структури, CRC і інкрементальний Parser: годуєте йому байти — він збирає кадр і перевіряє CRC.
Прочитати кадр — згодувати прочитані байти парсеру; коли feed() поверне true, у вас готовий пакет

2. Керувати дроном (слати CONTROL по UART)
Дрон не летить сам — ви ним кермуєте. Отримавши телеметрію, кожен такт шлете чекеру пакет PKT_CONTROL з двох нормованих чисел:
• accel — прискорення вздовж курсу, [-1..1]: 1 = повний газ, 0 = тримати швидкість, -1 = гальмо;
• turnRate — швидкість повороту, [-1..1]: +1 = вліво, -1 = вправо, 0 = прямо.
Окремо від логіки наведення (mission processor) реалізуйте свій модуль керування дроном, який перетворює рішення 
місії (куди летіти) на ці команди UART. Чекер за ними рухає дрон.
3. Сигнали на GPIO: START і DROP
Ваша програма керує двома лініями GPIO (через libgpiod):
• START — підняти у 1 одразу на старті й тримати. Сигнал чекеру «я готовий»: за ним чекер запускає симуляцію.
• DROP — короткий імпульс (50–100 мс) у момент скиду. Одноразово.

## Бінарний протокол UART (drone_link.h)
Усі multi-byte поля — little-endian. Кадр самосинхронізується за сигнатурою.
Кадр: A5 5A | TYPE(1) | LEN(1) | PAYLOAD(LEN) | CRC16(2, LE)
CRC16/CCITT-FALSE рахується по TYPE+LEN+PAYLOAD.
Чекер -> студент: 0x01 TELEMETRY 0x02 TARGET 0x03 AMMO
Студент -> чекер: 0x05 CONTROL
# PKT_TELEMETRY (0x01) — стан дрона (чекер -> студент):
Поле   | Тип      | Зміст
t_ms   | uint32   | час від старту, мс (таймстемп)
x, y   | float×2  |  позиція дрона в площині, м
z      | float    | висота (altitude), м
vx, vy | float×2  |  швидкість у площині, м/с
speed  | float    | модуль горизонтальної швидкості, м/с
dir    | float    |  курс польоту, радіани
state  | uint8    | стан стейт-машини (0..4)
# PKT_TARGET (0x02) — позиція цілі (чекер -> студент):
Поле  | Тип      | Зміст
id    | uint8    | індекс цілі
x, y  |  float×2 |  поточна позиція цілі, м
# PKT_AMMO (0x03) — раз на старті (чекер -> студент):
Поле              | Тип       | Зміст 
name              | char[16]  | назва боєприпасу (напр. VOG-17)
mass, drag, lift  | float×3   |  параметри балістики m, d, l
hitRadius         |  float    | радіус успішного влучання, м
nTargets          |  uint8    | кількість цілей у місії
# PKT_CONTROL (0x05) — команда керування (студент -> чекер):
Поле     | Тип    | Зміст
accel    |  float |  прискорення вздовж курсу, [-1..1]
turnRate |  float |  швидкість повороту, [-1..1] (+ вліво)

## Програма-чекер 
Чекер(надається) — окрема консольна програма. Запускається з номером 
тесту (місії) від 1 до 10:
      ./checker <N> # N = 1..10
# Що робить чекер:
• завантажує місію №N: старт дрона, його ліміти (maxSpeed, maxAccel, maxTurnRate), 
цілі та боєприпас;
• чекає на лінії START. Поки START = 0 — нічого не шле;
• щойно START = 1 — шле AMMO, далі кожен такт інтегрує фізику дрона за вашими 
CONTROL-командами і шле назад TELEMETRY і TARGET;
• стежить за лінією DROP. На імпульсі рахує балістику від поточного стану і 
позицію цілі на момент падіння;
• друкує вердикт: HIT(відстань ≤ hitRadius) або MISS, відстань промаху (м) і час скиду.

# Зараховується перший імпульс DROP; зайві ігноруються.

# Аргументи чекера:
Аргумент         | За замовч.         | Призначення
N                |      1             | номер тесту (місії), 1..10 — перший позиційний аргумент
--uart <dev>     |  /tmp/ttyB         | послідовний порт: туди чекер шле телеметрію, звідти читає CONTROL
--sim            | увімкнено          | режим симуляції: START/DROP читаються з gpio-sim через debugfs value
--hw             | —                  | режим реальної плати: START/DROP читаються через libgpiod (потрібна збірка з -DUSE_GPIOD)
--start-line <n> | 24 (sim) / 27 (hw) | номер лінії GPIO для сигналу START
--drop-line <n>  | 23 (sim) / 22 (hw) | номер лінії GPIO для сигналу DROP
--gpiochip <name>| gpiochip1          | тільки --hw: чип, на якому входи START/DROP чекера (на малині — gpiochip0)
--sim-bank <dir> | —                  | тільки sim: готовий debugfs-bank gpio-sim замість авто-створення чипа
--selftest       | —                  | прогнати референтний автопілот без UART/GPIO — перевірка, що місія розв'язна

## Запуск симуляції (без плати)
UART підміняється парою віртуальних портів (socat), GPIO — модулем gpio-sim. 
Спершу запускаєте чекер (він чекає на START), потім свою програму. Чекер друкує імʼя 
свого gpio-sim чипа — підставте його у --gpiochip студента:
  socat -d -d pty,raw,echo=0,link=/tmp/ttyA pty,raw,echo=0,link=/tmp/ttyB
  sudo ./checker 1 --uart /tmp/ttyB --start-line 24 --drop-line 23
  ./student --uart /tmp/ttyA --gpiochip gpiochipN --start-line 24 --drop-line 23

##  Реальна Raspberry Pi
Потрібно два UART на одній платі. Чекер слухає головний /dev/serial0 (піни 8/10). Для другого UART 
(ваша програма) додайте у /boot/firmware/config.txt рядок dtoverlay=uart3 і перезавантажте 
— зʼявиться /dev/ttyAMA1 (піни 7/29; перевірте: ls /dev/ttyAMA*).
GPIO — реальні піни. START і DROP закольцьовуємо перемичками: 
ваші виходи → входи чекера. Земля спільна (одна плата), усе на 3.3 В.

## Структура репо

```
homework_11/
├── include/                     TODO
│   ├── link/
│   │   ├── drone_link.h          # з ТЗ
│   │   ├── UartPort.hpp          # open/read/write через termios
│   │   └── DroneLink.hpp         # Parser + dispatch пакетів
│   ├── gpio/
│   │   └── GpioControl.hpp       # START/DROP через libgpiod
│   ├── autopilot/
│   │   ├── Autopilot.hpp         # головний цикл рішення
│   │   └── ControlMapper.hpp     # DroneCommand -> accel/turnRate
│   ├── mission/
│   |   ├── MissionProcessor.hpp
│   │   ├── MissionCtx.hpp
│   |   └── TargetControl.hpp
│   ├──  solvers/
|   |   ├── BallisticTable.hpp
|   |   └── TableSolver.hpp
│   └── math/
│       ├── angle_math.hpp
│       └── point_math.hpp
├── src/                           TODO
│   ├── main.cpp
│   ├── link/
│   ├── gpio/
│   └── autopilot/
│       ├── Autopilot.cpp         # головний цикл рішення
│       └── ControlMapper.cpp     # DroneCommand -> accel/turnRate
├── data/
│ ├── ammo.json
| ├── ballistic_table.txt
│ └── config.json
├── external/nlohmann
|  └── json.hpp   
├── README.md    
└── CMakeLists.txt

```
## Робоче середовище:

 Windows
└── VirtualBox
    └── Ubuntu VM
        ├── VS Code
        ├── Terminal
        ├── repo cpp-miltech
        ├── socat
        ├── gpio-sim
        └── checker + student program

 ## Кошмар-незабуть!

 VM: Ubuntu 24.04 у VirtualBox
Запуск: звичайний Start з GUI
Host key: Right Ctrl
Fullscreen toggle: Right Ctrl + F
Меню VirtualBox у fullscreen: Right Ctrl + Home
Paste в Ubuntu Terminal: Ctrl + Shift + V
Copy з Ubuntu Terminal: Ctrl + Shift + C       

* VM має Display: 128 MB, VMSVGA
* Якщо екран малий:
   - натиснути maximize/restore у вікні VirtualBox туди-назад
   - або Right Ctrl + G
   - або View → Auto-resize Guest Display

##  якщо gpio відсутні, їх треба створити

sudo modprobe gpio-sim
cd /sys/kernel/config/gpio-sim
sudo mkdir hw11chip
sudo mkdir hw11chip/bank0
echo 32 | sudo tee hw11chip/bank0/num_lines
echo 1 | sudo tee hw11chip/live
cat hw11chip/bank0/chip_name
gpiodetect

