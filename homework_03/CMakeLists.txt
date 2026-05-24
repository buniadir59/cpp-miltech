cmake_minimum_required(VERSION 3.20)
project(drone_simulation CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(hw3_ammo src/ammo.cpp)
target_include_directories(hw3_ammo PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(hw3_ammo PRIVATE -Wall -Wextra -pedantic)

add_library(hw3_point_math src/point_math.cpp)
target_include_directories(hw3_point_math PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(hw3_point_math PRIVATE -Wall -Wextra -pedantic)

add_library(hw3_anglemath src/anglemath.cpp)
target_include_directories(hw3_anglemath PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(hw3_anglemath PRIVATE -Wall -Wextra -pedantic)

add_library(hw3_ballistics src/ballistics.cpp)
target_include_directories(hw3_ballistics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(hw3_ballistics PRIVATE -Wall -Wextra -pedantic)
target_link_libraries(hw3_ballistics PUBLIC hw3_point_math hw3_anglemath)

add_library(hw3_drone src/drone.cpp)
target_include_directories(hw3_drone PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(hw3_drone PRIVATE -Wall -Wextra -pedantic)
target_link_libraries(hw3_drone PUBLIC hw3_point_math hw3_anglemath hw3_ammo hw3_ballistics)

add_library(hw3_sim src/simulation.cpp)
target_include_directories(hw3_sim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(hw3_sim PRIVATE -Wall -Wextra -pedantic)
target_link_libraries(hw3_sim PUBLIC hw3_point_math hw3_ammo hw3_drone)

add_executable(hw3_drone_sim src/main.cpp)
target_compile_options(hw3_drone_sim PRIVATE -Wall -Wextra -pedantic)
target_link_libraries(hw3_drone_sim PRIVATE hw3_sim)

# if(NOT CMAKE_CROSSCOMPILING) include(FetchContent) include(GoogleTest) FetchContent_Declare(
# googletest URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
# DOWNLOAD_EXTRACT_TIMESTAMP TRUE) FetchContent_MakeAvailable(googletest)
# add_executable(ballistics_tests tests/ballistics_tests.cpp) target_link_libraries(ballistics_tests
# PRIVATE ballistics GTest::gtest_main) gtest_discover_tests(ballistics_tests DISCOVERY_MODE
# PRE_TEST) endif()
