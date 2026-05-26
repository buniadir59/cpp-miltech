cmake_minimum_required(VERSION 3.20)
project(drone_simulation CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(hw7_ammo src/ammo.cpp)
target_include_directories(hw7_ammo PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(hw7_ammo PRIVATE -Wall -Wextra -pedantic)

add_library(hw7_point_math src/point_math.cpp)
target_include_directories(hw7_point_math PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(hw7_point_math PRIVATE -Wall -Wextra -pedantic)

add_library(hw7_anglemath src/anglemath.cpp)
target_include_directories(hw7_anglemath PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(hw7_anglemath PRIVATE -Wall -Wextra -pedantic)

add_library(hw7_ballistics src/ballistics.cpp)
target_include_directories(hw7_ballistics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(hw7_ballistics PRIVATE -Wall -Wextra -pedantic)
target_link_libraries(hw7_ballistics PUBLIC hw7_point_math hw7_anglemath)

add_library(hw7_drone src/drone.cpp)
target_include_directories(hw7_drone PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(hw7_drone PRIVATE -Wall -Wextra -pedantic)
target_link_libraries(hw7_drone PUBLIC hw7_point_math hw7_anglemath hw7_ammo hw7_ballistics)

add_library(hw7_sim src/simulation.cpp)
target_include_directories(hw7_sim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(hw7_sim PRIVATE -Wall -Wextra -pedantic)
target_link_libraries(hw7_sim PUBLIC hw7_point_math hw7_ammo hw7_drone)

add_executable(hw7_drone_sim src/main.cpp)
target_compile_options(hw7_drone_sim PRIVATE -Wall -Wextra -pedantic)
target_link_libraries(hw7_drone_sim PRIVATE hw7_sim)

# if(NOT CMAKE_CROSSCOMPILING) include(FetchContent) include(GoogleTest) FetchContent_Declare(
# googletest URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
# DOWNLOAD_EXTRACT_TIMESTAMP TRUE) FetchContent_MakeAvailable(googletest)
# add_executable(ballistics_tests tests/ballistics_tests.cpp) target_link_libraries(ballistics_tests
# PRIVATE ballistics GTest::gtest_main) gtest_discover_tests(ballistics_tests DISCOVERY_MODE
# PRE_TEST) endif()
