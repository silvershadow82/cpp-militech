cmake_minimum_required(VERSION 3.20)

project(rd_docker_run_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(rd51_run_demo src/main.cpp)
set_target_properties(rd51_run_demo PROPERTIES OUTPUT_NAME ugv-controller)
