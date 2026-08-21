cmake_minimum_required(VERSION 3.20)

# Окремий CMake-проєкт для домашньої роботи 10. Його можна збирати як через root CMakeLists.txt, так
# і напряму з цієї директорії.
project(reporter LANGUAGES CXX)

# Код домашньої роботи збирається з тим самим стандартом, що й лекційні приклади.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Add JSON library via CMake
include(FetchContent)

FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz
                              DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_MakeAvailable(json)

# --- Статична бібліотека з усіма продакшн-джерелами (окрім main.cpp) ---------
# Спільна кодова база для виконуваного файлу та тестів: компілюємо один раз.
add_library(reporter_lib STATIC src/reporter.cpp)
target_include_directories(reporter_lib PUBLIC include)
target_link_libraries(reporter_lib PUBLIC nlohmann_json::nlohmann_json)

# --- Виконуваний файл: тонкий main.cpp поверх бібліотеки -----------------------
add_executable(reporter src/main.cpp)
target_link_libraries(reporter PRIVATE reporter_lib)

# --- Тести (GoogleTest) -------------------------------------------------------
if(NOT CMAKE_CROSSCOMPILING)
    # Потрібен для standalone-збірки з цієї директорії: без нього CTestTestfile.cmake не
    # генерується і ctest не бачить тестів. При збірці через root CMakeLists.txt це no-op.
    enable_testing()

    include(GoogleTest)

    FetchContent_Declare(
        googletest URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
                       DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
    FetchContent_MakeAvailable(googletest)

    add_executable(reporter_tests tests/reporter_tests.cpp)
    target_link_libraries(reporter_tests PRIVATE reporter_lib GTest::gtest_main)
    target_compile_definitions(
        reporter_tests PRIVATE DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../data"
                               STATES_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src/states")

    gtest_discover_tests(reporter_tests DISCOVERY_MODE PRE_TEST)
endif()
