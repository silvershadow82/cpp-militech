cmake_minimum_required(VERSION 3.20)

# Coursework: visual target follow over MAVLink. Builds standalone or through the root CMakeLists.txt.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(follow LANGUAGES CXX)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)
find_package(Threads REQUIRED)

# --- Core: estimator, controller, supervisor. No I/O, threads or OpenCV. ------
add_library(
  follow_core STATIC
  src/core/Frames.cpp
  src/core/CameraModel.cpp
  src/core/AttitudeHistory.cpp
  src/core/TargetEstimator.cpp
  src/core/FollowController.cpp
  src/core/Supervisor.cpp
  src/core/Core.cpp)
target_include_directories(follow_core PUBLIC include)
target_compile_options(follow_core PRIVATE -Wall -Wextra)

# --- Simulation: synthetic camera, target and vehicle for closed-loop tests. --
add_library(
  follow_sim STATIC
  src/sim/SimTarget.cpp
  src/sim/SyntheticCamera.cpp
  src/sim/KinematicVehicle.cpp
  src/sim/ScenarioRunner.cpp
  src/sim/ScenarioCheck.cpp)
target_link_libraries(follow_sim PUBLIC follow_core)
target_compile_options(follow_sim PRIVATE -Wall -Wextra)

# --- Config: follow.json, camera and scenario files (nlohmann/json). ----------
# Same name and URL as homework_07..11, so the root build fetches the library once.
FetchContent_Declare(json URL https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz
                              DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_MakeAvailable(json)

add_library(follow_config STATIC src/config/ConfigJson.cpp src/config/ScenarioJson.cpp)
target_link_libraries(follow_config PUBLIC follow_sim nlohmann_json::nlohmann_json)
target_compile_options(follow_config PRIVATE -Wall -Wextra)

# --- Tests (GoogleTest, host only) --------------------------------------------
if(NOT CMAKE_CROSSCOMPILING)
  enable_testing()
  include(GoogleTest)

  FetchContent_Declare(googletest URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
                                      DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  FetchContent_MakeAvailable(googletest)

  add_executable(
    follow_tests
    tests/angles_frames_tests.cpp
    tests/camera_model_tests.cpp
    tests/attitude_history_tests.cpp
    tests/target_estimator_tests.cpp
    tests/follow_controller_tests.cpp
    tests/supervisor_tests.cpp
    tests/core_tests.cpp
    tests/sim_target_tests.cpp
    tests/synthetic_camera_tests.cpp
    tests/kinematic_vehicle_tests.cpp
    tests/scenario_tests.cpp
    tests/runtime_slots_tests.cpp
    tests/config_tests.cpp
    tests/scenario_json_tests.cpp)
  target_include_directories(follow_tests PRIVATE tests)
  target_link_libraries(follow_tests PRIVATE follow_config GTest::gtest_main Threads::Threads)
  target_compile_options(follow_tests PRIVATE -Wall -Wextra)
  target_compile_definitions(follow_tests PRIVATE FOLLOW_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
  gtest_discover_tests(follow_tests DISCOVERY_MODE PRE_TEST)
endif()
