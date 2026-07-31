cmake_minimum_required(VERSION 3.20)
project(logic_split_demo)

# C++20 - базовий стандарт курсу.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ament_auto бере залежності з package.xml і зменшує boilerplate.
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(decision_node src/decision_node.cpp)
target_include_directories(decision_node PRIVATE include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_auto_add_gtest(target_decision_test test/target_decision_test.cpp)
  target_include_directories(target_decision_test PRIVATE include)
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
