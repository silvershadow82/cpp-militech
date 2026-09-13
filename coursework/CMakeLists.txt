cmake_minimum_required(VERSION 3.20)

# Coursework: visual target follow over MAVLink. Builds standalone or through the root CMakeLists.txt.
if(NOT CMAKE_PROJECT_NAME)
  project(follow LANGUAGES CXX)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)

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
  src/sim/SimTarget.cpp)
target_link_libraries(follow_sim PUBLIC follow_core)
target_compile_options(follow_sim PRIVATE -Wall -Wextra)

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
    tests/sim_target_tests.cpp)
  target_include_directories(follow_tests PRIVATE tests)
  target_link_libraries(follow_tests PRIVATE follow_sim GTest::gtest_main)
  gtest_discover_tests(follow_tests DISCOVERY_MODE PRE_TEST)
endif()
