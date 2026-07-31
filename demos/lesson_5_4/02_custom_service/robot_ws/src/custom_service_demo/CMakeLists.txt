cmake_minimum_required(VERSION 3.20)
project(custom_service_demo)

# C++20 - базовий стандарт курсу.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ament_auto бере залежності з package.xml і зменшує boilerplate.
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Власний .srv генерує C++ типи Request і Response.
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/TriggerActuator.srv"
)

ament_auto_add_executable(trigger_server src/trigger_server.cpp)
ament_auto_add_executable(trigger_client src/trigger_client.cpp)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(trigger_server "${cpp_typesupport_target}")
target_link_libraries(trigger_client "${cpp_typesupport_target}")

ament_export_dependencies(rosidl_default_runtime)

ament_auto_package()
