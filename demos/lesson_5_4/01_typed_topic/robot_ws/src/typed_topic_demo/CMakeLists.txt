cmake_minimum_required(VERSION 3.20)
project(typed_topic_demo)

# C++20 - базовий стандарт курсу.
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ament_auto бере залежності з package.xml і прибирає зайвий шаблонний код.
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Власний .msg генерує C++ тип для типізованого топіка.
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Target.msg"
)

ament_auto_add_executable(target_publisher src/target_publisher.cpp)
ament_auto_add_executable(target_subscriber src/target_subscriber.cpp)

target_include_directories(target_subscriber PRIVATE include)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(target_publisher "${cpp_typesupport_target}")
target_link_libraries(target_subscriber "${cpp_typesupport_target}")

ament_export_dependencies(rosidl_default_runtime)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
