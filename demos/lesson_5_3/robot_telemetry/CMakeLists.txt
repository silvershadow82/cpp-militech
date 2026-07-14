cmake_minimum_required(VERSION 3.20)
project(robot_telemetry)

# C++20 - базовий стандарт курсу.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ament_auto бере залежності з package.xml і прибирає ручний шаблонний CMake-код.
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(telemetry_publisher src/telemetry_publisher.cpp)

ament_auto_add_executable(status_monitor src/status_monitor.cpp)

ament_auto_package()
