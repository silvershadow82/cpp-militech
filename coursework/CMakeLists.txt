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

# MAVLink C headers: the homework_11/c_library_v2 git submodule.
set(FOLLOW_MAVLINK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../homework_11/c_library_v2" CACHE PATH "MAVLink c_library_v2 checkout")

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

# --- MAVLink: byte links (UDP, UART) and the MAVLink client. ------------------
add_library(follow_mavlink STATIC src/mavlink/Links.cpp src/mavlink/MavlinkClient.cpp)
target_include_directories(follow_mavlink SYSTEM PUBLIC ${FOLLOW_MAVLINK_DIR})
target_link_libraries(follow_mavlink PUBLIC follow_core Threads::Threads)
target_compile_options(follow_mavlink PRIVATE -Wall -Wextra)

# --- Runtime: threads, shared slots, run log. ---------------------------------
add_library(
  follow_runtime STATIC
  src/runtime/RunLog.cpp
  src/runtime/ControlLoop.cpp
  src/runtime/MavlinkIo.cpp
  src/runtime/SimVision.cpp
  src/runtime/SimApp.cpp)
target_link_libraries(follow_runtime PUBLIC follow_config follow_mavlink Threads::Threads)
target_compile_options(follow_runtime PRIVATE -Wall -Wextra)

# --- Vision: camera, tracker and overlay (OpenCV). ----------------------------
# Off by default: the devcontainer has no OpenCV and the aarch64 toolchain has no sysroot, so both
# CI builds must keep working without it. Turn on for a native build where OpenCV is installed.
option(FOLLOW_WITH_OPENCV "Build the OpenCV vision adapter and tools" OFF)

if(FOLLOW_WITH_OPENCV)
  find_package(OpenCV CONFIG REQUIRED)
  message(STATUS "follow: OpenCV ${OpenCV_VERSION} from ${OpenCV_INSTALL_PATH}")

  add_library(follow_vision STATIC src/vision/Tracker.cpp src/vision/FrameSource.cpp src/vision/PiCameraSource.cpp
                                    src/vision/CameraTrackerSource.cpp src/vision/Overlay.cpp src/vision/HwApp.cpp)
  target_link_libraries(follow_vision PUBLIC follow_runtime ${OpenCV_LIBS})
  target_compile_options(follow_vision PRIVATE -Wall -Wextra)
endif()

# --- Executables ----------------------------------------------------------------
add_executable(follow_app src/app/main.cpp)
target_link_libraries(follow_app PRIVATE follow_runtime)
target_compile_options(follow_app PRIVATE -Wall -Wextra)
if(FOLLOW_WITH_OPENCV)
  target_link_libraries(follow_app PRIVATE follow_vision)
  target_compile_definitions(follow_app PRIVATE FOLLOW_WITH_OPENCV)
endif()

add_executable(follow_check_run src/app/check_run_main.cpp)
target_link_libraries(follow_check_run PRIVATE follow_runtime)
target_compile_options(follow_check_run PRIVATE -Wall -Wextra)

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
    tests/scenario_json_tests.cpp
    tests/links_tests.cpp
    tests/mavlink_client_tests.cpp
    tests/run_log_tests.cpp
    tests/control_loop_tests.cpp
    tests/mavlink_io_tests.cpp
    tests/sim_vision_tests.cpp
    tests/sim_app_tests.cpp)
  if(FOLLOW_WITH_OPENCV)
    target_sources(follow_tests PRIVATE tests/vision_tracker_tests.cpp tests/vision_frame_source_tests.cpp
                                        tests/vision_pi_camera_tests.cpp tests/vision_camera_tracker_source_tests.cpp
                                        tests/vision_overlay_tests.cpp)
    target_link_libraries(follow_tests PRIVATE follow_vision)
  endif()
  target_include_directories(follow_tests PRIVATE tests)
  target_link_libraries(follow_tests PRIVATE follow_runtime GTest::gtest_main)
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_link_libraries(follow_tests PRIVATE util) # openpty
  endif()
  target_compile_options(follow_tests PRIVATE -Wall -Wextra)
  target_compile_definitions(follow_tests PRIVATE FOLLOW_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
  gtest_discover_tests(follow_tests DISCOVERY_MODE PRE_TEST)
endif()
