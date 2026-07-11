cmake_minimum_required(VERSION 3.20)
project(drone_simulation CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(
    hw10_core
    src/math/point_math.cpp
    src/math/angle_math.cpp
    src/config/FileConfigLoader.cpp
    src/mission/MissionProcessor.cpp
    src/mission/TargetControl.cpp
    src/config/ComponentFactory.cpp
    src/solvers/TableSolver.cpp
    src/autopilot/autopilot.cpp
    src/mission/MissionCtx.cpp
    )

target_include_directories(
    hw11_core
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/external)
target_compile_options(hw11_core PRIVATE -Wall -Wextra -pedantic)

add_executable(hw11_drone_sim src/main.cpp)
target_include_directories(hw11_drone_sim PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
                                                 ${CMAKE_CURRENT_SOURCE_DIR}/external)
target_link_libraries(hw11_drone_sim PRIVATE hw11_core)
