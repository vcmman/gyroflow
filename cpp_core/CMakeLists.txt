cmake_minimum_required(VERSION 3.16)

project(gyroflow_cpp_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)

add_library(gyroflow_cpp_core
    src/lens_profile.cpp
    src/quaternion.cpp
    src/smoothing.cpp
    src/stabilization.cpp
    src/distortion/opencv_fisheye.cpp
    src/telemetry_io.cpp
    src/smoothing/crop_agc.cpp
    src/smoothing/default_algo.cpp
    src/stabilization/frame_transform.cpp
    src/stabilization/undistort.cpp
    src/zooming/adaptive_zoom.cpp
)

target_include_directories(gyroflow_cpp_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party
)

target_compile_features(gyroflow_cpp_core PUBLIC cxx_std_17)
target_link_libraries(gyroflow_cpp_core PUBLIC Threads::Threads)

add_executable(gyroflow_cpp_probe tools/gyroflow_cpp_probe.cpp)
target_link_libraries(gyroflow_cpp_probe PRIVATE gyroflow_cpp_core)

# Validation dumper (smoothed quats + adaptive FOVs) for diffing against Rust Gyroflow's
# exported metadata. OpenCV-free — links the core library only.
add_executable(gyroflow_cpp_validate tools/gyroflow_cpp_validate.cpp)
target_link_libraries(gyroflow_cpp_validate PRIVATE gyroflow_cpp_core)

# Headless DJI stabilizer CLI (OpenCV video I/O). Built only when OpenCV is available.
find_package(OpenCV QUIET COMPONENTS core imgproc videoio)
if(OpenCV_FOUND)
    add_executable(gyroflow_cpp_stabilize tools/gyroflow_cpp_stabilize.cpp)
    target_link_libraries(gyroflow_cpp_stabilize PRIVATE gyroflow_cpp_core ${OpenCV_LIBS})
    target_include_directories(gyroflow_cpp_stabilize PRIVATE ${OpenCV_INCLUDE_DIRS})
    message(STATUS "gyroflow_cpp_stabilize: enabled (OpenCV ${OpenCV_VERSION})")
else()
    message(STATUS "gyroflow_cpp_stabilize: disabled (OpenCV not found)")
endif()

enable_testing()

foreach(test_name core fisheye telemetry_io transform adaptive_zoom imu_orientation smoothing)
    add_executable(gyroflow_cpp_${test_name}_tests tests/test_${test_name}.cpp)
    target_link_libraries(gyroflow_cpp_${test_name}_tests PRIVATE gyroflow_cpp_core)
    add_test(NAME gyroflow_cpp_${test_name}_tests COMMAND gyroflow_cpp_${test_name}_tests)
endforeach()
