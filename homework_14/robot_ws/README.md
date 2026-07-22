## Мета

ДЗ 14 має поєднати C++ алгоритм, ROS 2 topic-и, сервіси, launch і доказовий запис у rosbag.

Сценарій: робот-собака заходить у систему траншей і бліндажів, подану як клітинкова схема (grid-world), 
через один вхід / стартову клітинку. Повна карта прихована. Надається тільки underground_world_node, 
який знає світ, але публікує системі рішення лише локальний огляд навколо поточної клітинки. 

Потрібно дослідити доступну прохідну частину карти, обробити всі знайдені контакти через 
симульований сервіс дії і записати проходження в rosbag.

Карта потрібна як внутрішня пам'ять алгоритму, щоб не ходити наосліп і не повторювати вже досліджені 
коридори.



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
    ├── CMakeLists.txt             # find_package(underground_world)
    ├── include/
    │   └── mission_explorer/
    │       ├── grid_types.hpp
    │       └── explorer_core.hpp
    └── src/
        ├── explorer_core.cpp
        ├── mission_explorer_node.cpp
        └── payload_action_node.cpp

## Запуск

ros2 launch underground_world system.launch.py \
  scenario:=small_rooms.yaml

## 