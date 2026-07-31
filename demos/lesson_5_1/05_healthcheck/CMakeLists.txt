cmake_minimum_required(VERSION 3.20)

project(rd51_health_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(rd51_health_service src/service.cpp)
add_executable(rd51_health_check src/health_check.cpp)
add_executable(rd51_healthctl src/healthctl.cpp)
