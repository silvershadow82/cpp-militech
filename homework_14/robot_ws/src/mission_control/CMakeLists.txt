cmake_minimum_required(VERSION 3.20)
project(mission_control)

# C++20 - базовий стандарт курсу.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(mission_control_core
  src/grid.cpp
  src/map_memory.cpp
  src/explorer.cpp
)
target_include_directories(mission_control_core PUBLIC include)

ament_auto_add_executable(mission_explorer_node src/mission_explorer_node.cpp)
target_link_libraries(mission_explorer_node mission_control_core)

ament_auto_add_executable(payload_action_node src/payload_action_node.cpp)
target_link_libraries(payload_action_node mission_control_core)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(mission_control_grid_test test/grid_test.cpp)
    target_link_libraries(mission_control_grid_test mission_control_core)

    ament_add_gtest(mission_control_map_memory_test test/map_memory_test.cpp)
    target_link_libraries(mission_control_map_memory_test mission_control_core)

    ament_add_gtest(mission_control_explorer_test test/explorer_test.cpp)
    target_link_libraries(mission_control_explorer_test mission_control_core)

    # Наданий пакет експортує underground_world_core лише як шлях до бібліотеки,
    # без CMake-таргета, тому ament_target_dependencies його не підхоплює -
    # шукаємо за тією ж логікою, що й ament_cmake_export_libraries.
    find_library(underground_world_core_LIBRARY
        NAMES underground_world_core
        PATHS "${underground_world_DIR}/../../../lib"
        NO_DEFAULT_PATH
        REQUIRED
    )

    # Прогін алгоритму проти справжнього world node з наданого пакета.
    ament_add_gtest(mission_control_scenario_sim_test test/scenario_sim_test.cpp)
    target_link_libraries(mission_control_scenario_sim_test
        mission_control_core
        ${underground_world_core_LIBRARY}
    )
    ament_target_dependencies(mission_control_scenario_sim_test underground_world ament_index_cpp)
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
