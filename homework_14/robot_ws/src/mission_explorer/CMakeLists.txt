cmake_minimum_required(VERSION 3.20)
project(underground_world)

# C++20 - базовий стандарт курсу.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ament_auto бере залежності з package.xml і тримає CMake коротшим.
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Власні msg/srv описують контракт між world node і керуючою системою.
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CellObservation.msg"
  "msg/LocalScan.msg"
  "msg/MoveCommand.msg"
  "msg/EnemyDown.msg"
  "msg/RobotMetrics.msg"
  "msg/RobotResult.msg"
  "msg/StudentStatus.msg"
  "srv/PayloadTrigger.srv"
)

ament_auto_add_library(mission_explorer_core
  src/explorer_core.cpp
)
target_include_directories(mission_explorer_core PUBLIC include)

ament_auto_add_executable(mission_explorer_node src/mission_explorer_node.cpp)
target_link_libraries(mission_explorer_node mission_explorer_core)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)
target_link_libraries(mission_explorer_node "${cpp_typesupport_target}")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(explorer_core_test
    test/explorer_core_test.cpp
  )
  target_include_directories(explorer_core_test PRIVATE include)
  target_link_libraries(explorer_core_test mission_explorer_core)
  target_compile_definitions(explorer_core_test
    PRIVATE UNDERGROUND_WORLD_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
  )
endif()

ament_export_dependencies(rosidl_default_runtime)

# launch/ і config/ встановлюються у share, щоб ros2 launch знаходив сценарії.
ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR INSTALL_TO_SHARE
  launch
  config
)
