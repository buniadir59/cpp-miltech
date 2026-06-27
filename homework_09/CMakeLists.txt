cmake_minimum_required(VERSION 3.20)
project(drone_simulation CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(
    hw9_core
    src/math/point_math.cpp
    src/math/angle_math.cpp
    src/config/ManualSimulationClock.cpp
    src/providers/JsonTargetProvider.cpp
    src/solvers/AnalyticalSolver.cpp
    src/config/FileConfigLoader.cpp
    src/core/MissionProcessor.cpp
    src/core/DroneControl.cpp
    src/core/TargetControl.cpp
    src/config/ComponentFactory.cpp
    src/solvers/TableSolver.cpp
    src/drone/DroneContext.cpp
    src/drone/Stopped.cpp
    src/drone/Turning.cpp
    src/drone/Accelerating.cpp
    src/drone/Decelerating.cpp
    src/drone/Moving.cpp
    src/mission/MissionCtx.cpp
    src/mission/Idle.cpp
    src/mission/Attack.cpp
    )

target_include_directories(
    hw9_core
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/external)
target_compile_options(hw9_core PRIVATE -Wall -Wextra -pedantic)

add_executable(hw9_drone_sim src/main.cpp)
target_include_directories(hw9_drone_sim PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
                                                 ${CMAKE_CURRENT_SOURCE_DIR}/external)
target_link_libraries(hw9_drone_sim PRIVATE hw9_core)
