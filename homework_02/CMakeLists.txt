cmake_minimum_required(VERSION 3.20)
project(drone_simulation CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(hw2_point_math src/point_math.cpp)
target_include_directories(hw2_point_math PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(hw2_point_math PRIVATE -Wall -Wextra -pedantic)

add_library(hw2_drone src/drone.cpp)
target_include_directories(hw2_drone PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(hw2_drone PRIVATE -Wall -Wextra -pedantic)
target_link_libraries(hw2_drone PUBLIC hw2_point_math ballistics)

add_library(hw2_sim src/simulation.cpp)
target_include_directories(hw2_sim PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(hw2_sim PRIVATE -Wall -Wextra -pedantic)
target_link_libraries(hw2_sim PUBLIC hw2_point_math hw2_drone)

add_executable(hw2_drone_sim src/main.cpp)
target_compile_options(hw2_drone_sim PRIVATE -Wall -Wextra -pedantic)
target_link_libraries(hw2_drone_sim PRIVATE hw2_sim)


#if(NOT CMAKE_CROSSCOMPILING)
#    include(FetchContent)
#    include(GoogleTest)
#    FetchContent_Declare(
#        googletest URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
#                       DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
#    FetchContent_MakeAvailable(googletest)
#    add_executable(ballistics_tests tests/ballistics_tests.cpp)
#    target_link_libraries(ballistics_tests PRIVATE ballistics GTest::gtest_main)
#    gtest_discover_tests(ballistics_tests DISCOVERY_MODE PRE_TEST)
#endif()
