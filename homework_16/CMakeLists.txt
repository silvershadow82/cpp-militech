cmake_minimum_required(VERSION 3.20)

# Окремий CMake-проєкт для домашньої роботи 15. Його можна збирати як через root CMakeLists.txt, так
# і напряму з цієї директорії.
project(i2c LANGUAGES CXX)

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
add_library(i2chardware_lib STATIC src/i2c_device.cpp src/bmp388.cpp)
target_include_directories(i2chardware_lib PUBLIC include)
target_link_libraries(i2chardware_lib PUBLIC nlohmann_json::nlohmann_json)

# --- Виконуваний файл: тонкий main.cpp поверх бібліотеки -----------------------
add_executable(i2chardware src/main.cpp)
target_link_libraries(i2chardware PRIVATE i2chardware_lib)

# --- Тести (GoogleTest) -------------------------------------------------------
if(NOT CMAKE_CROSSCOMPILING)
    include(GoogleTest)

    FetchContent_Declare(
        googletest URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
                       DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
    FetchContent_MakeAvailable(googletest)

    add_executable(i2chardware_tests tests/i2chardware_tests.cpp)
    target_link_libraries(i2chardware_tests PRIVATE i2chardware_lib GTest::gtest_main)

    gtest_discover_tests(i2chardware_tests DISCOVERY_MODE PRE_TEST)
endif()
