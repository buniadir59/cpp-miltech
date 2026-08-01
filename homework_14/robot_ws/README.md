## Мета

ДЗ 14 має поєднати C++ алгоритм, ROS 2 topic-и, сервіси, launch і доказовий запис у rosbag.

Сценарій: робот-собака заходить у систему траншей і бліндажів, подану як клітинкова схема (grid-world), 
через один вхід / стартову клітинку. Повна карта прихована. Надається тільки underground_world_node, 
який знає світ, але публікує системі рішення лише локальний огляд навколо поточної клітинки. 

Потрібно дослідити доступну прохідну частину карти, обробити всі знайдені контакти через 
симульований сервіс дії і записати проходження в rosbag.

Карта потрібна як внутрішня пам'ять алгоритму, щоб не ходити наосліп і не повторювати вже досліджені 
коридори.

## Критерії приймання

 - colcon build --symlink-install проходить.
 - Обов'язкові сценарії завершуються з mission_result=SUCCESS.
 - Разом із mission_result=SUCCESS виконується contacts_seen == contacts_down.
 - map_coverage_percent досягає 100% для доступної прохідної частини сценарію.
 - /robot/metrics не показує зайві помилки проходження: invalid_moves=0, invalid_triggers=0, 
 duplicate_triggers=0 для базового зарахування.
 - mission_result=SUCCESS означає, що бюджет команд max_steps не вичерпано.
 - /student/status публікується під час кожного запуску.
 - Rosbag здається для кожного обов'язкового сценарію.

# Що оцінюється
Базово оцінюється коректність:

  система стартує через launch;
  ROS 2 контракти виконані;
  робот досягає умови успіху;
  контакти оброблені;
  фінальні значення /robot/metrics відповідають успішному проходженню;
  доказові rosbag-записи є.

Швидкість і якість маршруту можуть використовуватися для ручного перегляду або бонусу:

  менше steps_taken;
  менше повторних відвідувань;
  нуль invalid_moves;
  нуль invalid_triggers;
  нуль duplicate_triggers;
  повніше покриття карти;
  повернення робота у стартову клітинку S після виконання базової місії.

Назва алгоритму не оцінюється. Оцінюється поведінка у сценаріях і доказовий запис у rosbag.



## Структура

robot_ws/src/
├── underground_world/             # вхідний код проекту (скелет), зміна тільки в лонч-файлі
│   ├── msg/
│   ├── srv/
│   ├── src/
│   │   └── underground_world_node.cpp
│   └── launch/
│       └── system.launch.py       # запускає всі три ноди
│
└── mission_explorer/
    ├── package.xml                # depend: underground_world
    ├── CMakeLists.txt             # залежності автоматично знаходяться через ament_cmake_auto
    ├── include/
    │   └── mission_explorer/
    │       └── explorer_core.hpp
    └── src/
        ├── explorer_core.cpp
        ├── mission_explorer_node.cpp
        └── payload_action_node.cpp

## Bild

colcon build --symlink-install
source install/setup.bash

## Запуск

1) перший термінал - запустити запис в rosbag:
ros2 bag record -a -o ../bags/small_rooms

де -a означає all topics.

2) Другий термінал:
ros2 launch underground_world system.launch.py \
  scenario:=small_rooms.yaml
  
## Перевірка


ros2 bag info ../bags/small_rooms

## Відтворення

ros2 bag play ../bags/small_rooms

# Щоб listener можна було запустити першим, задайте тип явно:

ros2 topic echo \
  /robot/result \
  underground_world/msg/RobotResult

