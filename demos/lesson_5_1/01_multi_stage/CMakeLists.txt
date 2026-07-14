cmake_minimum_required(VERSION 3.20)

project(rd_multi_stage_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(rd51_multi_stage_app src/main.cpp)
set_target_properties(rd51_multi_stage_app PROPERTIES OUTPUT_NAME app)
