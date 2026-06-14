cmake_minimum_required(VERSION 3.20)
project(drone_simulation CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(
    hw8_core
    src/math/point_math.cpp
    src/math/angle_math.cpp
    src/config/ManualSimulationClock.cpp
    src/providers/JsonTargetProvider.cpp
    src/solvers/AnalyticalSolver.cpp
    src/config/FileConfigLoader.cpp
    src/MissionProcessor.cpp
    src/DroneControl.cpp
    src/Mission.cpp
    src/TargetControl.cpp
    src/config/ComponentFactory.cpp)

target_include_directories(
    hw8_core
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/external)
target_compile_options(hw8_core PRIVATE -Wall -Wextra -pedantic)

add_executable(hw8_drone_sim src/main.cpp)
target_include_directories(hw8_drone_sim PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
                                                 ${CMAKE_CURRENT_SOURCE_DIR}/external)
target_link_libraries(hw8_drone_sim PRIVATE hw8_core)
