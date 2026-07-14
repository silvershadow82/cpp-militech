cmake_minimum_required(VERSION 3.20)
project(launch_demo)

# C++20 - базовий стандарт курсу.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ament_auto бере залежності з package.xml і зменшує boilerplate.
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(sensor_process src/sensor_process.cpp)
ament_auto_add_executable(controller_process src/controller_process.cpp)

# launch/ і config/ встановлюються у share, щоб ros2 launch знаходив файли.
ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
