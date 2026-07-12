cmake_minimum_required(VERSION 3.20)

# Окремий CMake-проєкт для домашньої роботи 11. Його можна збирати як через root CMakeLists.txt, так
# і напряму з цієї директорії.
project(simulation LANGUAGES CXX)

# Код домашньої роботи збирається з тим самим стандартом, що й лекційні приклади.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Багатопотокова симуляція вимагає pthread (std::thread / std::mutex / ...).
find_package(Threads REQUIRED)

# Інструментація ThreadSanitizer лише для таргетів >= homework_10.
option(SIMULATION11_TSAN "Build homework_11 targets with ThreadSanitizer" OFF)

# Опція використовувати реальний libgpiod, бо на Маку не працює збірка
option(USE_GPIOD "Use actual libgpiod" OFF)

# Add JSON library via CMake
include(FetchContent)

FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz
                              DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_MakeAvailable(json)

# --- Статична бібліотека з усіма продакшн-джерелами (окрім main.cpp) ---------
# Спільна кодова база для виконуваного файлу та тестів: компілюємо один раз.
add_library(
    simulation11_lib STATIC
    src/solvers/AnalyticalSolver.cpp
    src/solvers/TableSolver.cpp
    src/config/ComponentFactory.cpp
    src/config/FileConfigLoader.cpp
    src/config/UartConfigLoader.cpp
    src/providers/ThreadSafeTargetProvider.cpp
    src/providers/UartTargetProvider.cpp
    src/models/BallisticTable.cpp
    src/models/Coord.cpp
    src/models/FireGeometry.cpp
    src/control/FlightController.cpp
    src/gpio/LibGpiodController.cpp
    src/states/StateAccelerating.cpp
    src/states/StateDecelerating.cpp
    src/states/StateMoving.cpp
    src/states/StateStopped.cpp
    src/states/StateTurning.cpp
    src/DronePhysics.cpp
    src/StatCollector.cpp
    src/MissionProcessor.cpp
    src/util.cpp
    src/comms/SerialLink.cpp)
target_include_directories(simulation11_lib PUBLIC include)
target_link_libraries(simulation11_lib PUBLIC nlohmann_json::nlohmann_json Threads::Threads)

if(USE_GPIOD)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(LIBGPIOD REQUIRED IMPORTED_TARGET libgpiod)
    target_link_libraries(simulation11_lib PUBLIC PkgConfig::LIBGRPIOD)
endif()

# Шлях до балістичної таблиці зашитий у бібліотеку (використовує лише TableSolver.cpp). PRIVATE, щоб
# не проштовхувати макрос у тести (вони визначають власний).
target_compile_definitions(
    simulation11_lib
    PRIVATE BALLISTIC_TABLE_PATH="${CMAKE_CURRENT_BINARY_DIR}/table/ballistic_table.txt")

# --- Виконуваний файл: тонкий main.cpp поверх бібліотеки -----------------------
add_executable(simulation11 src/main.cpp)
target_link_libraries(simulation11 PRIVATE simulation11_lib)

add_custom_command(
    TARGET simulation11
    POST_BUILD
    COMMAND
        ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_CURRENT_SOURCE_DIR}/../data/table/ballistic_table.txt
        ${CMAKE_CURRENT_BINARY_DIR}/table/ballistic_table.txt
    COMMENT "Copying ballistic table to build directory")

# --- Тести (GoogleTest) -------------------------------------------------------
if(NOT CMAKE_CROSSCOMPILING)
    include(GoogleTest)

    FetchContent_Declare(
        googletest URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
                       DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
    FetchContent_MakeAvailable(googletest)

    add_executable(simulation11_tests tests/simulation_tests.cpp)
    target_link_libraries(simulation11_tests PRIVATE simulation11_lib GTest::gtest_main
                                                     Threads::Threads)
    target_compile_definitions(
        simulation11_tests
        PRIVATE BALLISTIC_TABLE_PATH="${CMAKE_CURRENT_BINARY_DIR}/table/ballistic_table.txt"
                DATA_BASIC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../data/basic"
                STATES_SRC_DIR="${CMAKE_CURRENT_SOURCE_DIR}/src/states")

    # Таблиця має бути й поруч із тестовим бінарником — деякі тести залежать від неї.
    add_custom_command(
        TARGET simulation11_tests
        POST_BUILD
        COMMAND
            ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/../data/table/ballistic_table.txt
            ${CMAKE_CURRENT_BINARY_DIR}/table/ballistic_table.txt
        COMMENT "Copying ballistic table to tests build directory")

    gtest_discover_tests(simulation11_tests DISCOVERY_MODE PRE_TEST)
endif()

# --- ThreadSanitizer (лише для таргетів >= homework_10) --------------------------
# Не додаємо глобальних прапорців, щоб не інструментувати homework_04..08.
if(SIMULATION11_TSAN)
    set(SIMULATION11_TSAN_FLAGS -fsanitize=thread -fno-omit-frame-pointer -g -O1)
    foreach(sim11_target simulation11_lib simulation11 simulation11_tests)
        if(TARGET ${sim11_target})
            target_compile_options(${sim11_target} PRIVATE ${SIMULATION11_TSAN_FLAGS})
            target_link_options(${sim11_target} PRIVATE ${SIMULATION11_TSAN_FLAGS})
        endif()
    endforeach()
endif()
