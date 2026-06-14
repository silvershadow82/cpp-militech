cmake_minimum_required(VERSION 3.20)

# Окремий CMake-проєкт для домашньої роботи 07. Його можна збирати як через root CMakeLists.txt, так
# і напряму з цієї директорії.
project(simulation LANGUAGES CXX)

# Код домашньої роботи збирається з тим самим стандартом, що й лекційні приклади.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Add JSON library via CMake
include(FetchContent)

FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz
                              DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_MakeAvailable(json)

# add_library(ballistics src/ballistics.cpp)
add_executable(
    simulation8
    src/main.cpp
    src/solvers/AnalyticalSolver.cpp
    src/config/ComponentFactory.cpp
    src/config/FileConfigLoader.cpp
    src/providers/JsonTargetProvider.cpp
    src/models/Coord.cpp
    src/models/Target.cpp
    src/StatCollector.cpp
    src/MissionProcessor.cpp
    src/util.cpp)
target_include_directories(simulation8 PUBLIC include)
target_link_libraries(simulation8 PRIVATE nlohmann_json::nlohmann_json)

if(NOT CMAKE_CROSSCOMPILING)
    include(GoogleTest)

    FetchContent_Declare(
        googletest URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
                       DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
    FetchContent_MakeAvailable(googletest)

    add_executable(simulation8_tests tests/simulation_tests.cpp)
    target_link_libraries(simulation8_tests PRIVATE GTest::gtest_main)

    gtest_discover_tests(simulation8_tests DISCOVERY_MODE PRE_TEST)
endif()
