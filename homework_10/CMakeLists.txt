cmake_minimum_required(VERSION 3.20)
project(drone_simulation CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(
    hw10_core
    src/math/point_math.cpp
    src/math/angle_math.cpp
    src/providers/ThreadSafeTargetProvider.cpp
    src/solvers/AnalyticalSolver.cpp
    src/config/FileConfigLoader.cpp
    src/core_/MissionProcessor.cpp
    src/core_/TargetControl.cpp
    src/config/ComponentFactory.cpp
    src/solvers/TableSolver.cpp
    src/drone/DronePhysics.cpp
    src/mission/MissionCtx.cpp
    )

target_include_directories(
    hw10_core
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/external)
target_compile_options(hw10_core PRIVATE -Wall -Wextra -pedantic)

add_executable(hw10_drone_sim src/main.cpp)
target_include_directories(hw10_drone_sim PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
                                                 ${CMAKE_CURRENT_SOURCE_DIR}/external)
target_link_libraries(hw10_drone_sim PRIVATE hw10_core)
