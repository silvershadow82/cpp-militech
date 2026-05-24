cmake_minimum_required(VERSION 3.20)

project(telemetry_check LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Create telemetry library
add_library(telemetry SHARED src/telemetry.cpp)
target_include_directories(telemetry PUBLIC include)

# Create main executable
add_executable(telemetry_check src/main.cpp)

# Link the library to the executable
target_link_libraries(telemetry_check PRIVATE telemetry)

target_compile_options(telemetry PRIVATE -Wall)
target_compile_options(telemetry_check PRIVATE -Wall -Wextra -pedantic)
