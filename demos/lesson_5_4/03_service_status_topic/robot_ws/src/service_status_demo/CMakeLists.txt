cmake_minimum_required(VERSION 3.20)
project(service_status_demo)

# C++20 - базовий стандарт курсу.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ament_auto бере залежності з package.xml і зменшує boilerplate.
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Власні msg/srv генерують C++ типи для status topic і service contract.
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ActuatorStatus.msg"
  "srv/TriggerActuator.srv"
)

ament_auto_add_executable(actuator_server src/actuator_server.cpp)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(actuator_server "${cpp_typesupport_target}")

ament_export_dependencies(rosidl_default_runtime)

# launch/ і config/ встановлюються у share, щоб ros2 launch знаходив файли.
ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
