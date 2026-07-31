cmake_minimum_required(VERSION 3.20)
project(parameters_demo)

# C++20 - базовий стандарт курсу.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ament_auto бере залежності з package.xml і зменшує boilerplate.
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(parameter_reader src/parameter_reader.cpp)

# config/ встановлюється у share, щоб ros2 run/launch могли отримати шлях до YAML.
ament_auto_package(INSTALL_TO_SHARE
  config
)
