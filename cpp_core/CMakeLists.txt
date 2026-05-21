cmake_minimum_required(VERSION 3.16)

project(gyroflow_cpp_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(gyroflow_cpp_core
    src/lens_profile.cpp
    src/quaternion.cpp
    src/smoothing.cpp
    src/stabilization.cpp
)

target_include_directories(gyroflow_cpp_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(gyroflow_cpp_core PUBLIC cxx_std_17)

add_executable(gyroflow_cpp_probe tools/gyroflow_cpp_probe.cpp)
target_link_libraries(gyroflow_cpp_probe PRIVATE gyroflow_cpp_core)

enable_testing()

add_executable(gyroflow_cpp_core_tests tests/test_core.cpp)
target_link_libraries(gyroflow_cpp_core_tests PRIVATE gyroflow_cpp_core)
add_test(NAME gyroflow_cpp_core_tests COMMAND gyroflow_cpp_core_tests)
