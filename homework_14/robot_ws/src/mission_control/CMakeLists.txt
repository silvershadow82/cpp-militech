cmake_minimum_required(VERSION 3.20)
project(mission_control)

# C++20 - базовий стандарт курсу.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(mission_control_core src/grid.cpp)
target_include_directories(mission_control_core PUBLIC include)

ament_auto_add_executable(mission_explorer_node src/mission_explorer_node.cpp)
target_link_libraries(mission_explorer_node mission_control_core)

ament_auto_add_executable(payload_action_node src/payload_action_node.cpp)
target_link_libraries(payload_action_node mission_control_core)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(mission_control_grid_test test/grid_test.cpp)
    target_link_libraries(mission_control_grid_test mission_control_core)
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
