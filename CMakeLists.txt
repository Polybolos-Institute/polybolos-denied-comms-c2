cmake_minimum_required(VERSION 3.16)
project(polybolos-sensor-fusion-framework CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(FUSION_BUILD_TESTS "Build unit tests (GoogleTest)" ON)
option(FUSION_BUILD_BENCHMARKS "Build benchmarks" ON)

# Eigen (header-only). Prefer vendored third_party/eigen.
set(FUSION_EIGEN_DIR "${CMAKE_SOURCE_DIR}/third_party/eigen")
if(NOT EXISTS "${FUSION_EIGEN_DIR}/Eigen/Core")
    include(FetchContent)
    FetchContent_Declare(
        eigen
        URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip
        DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    )
    FetchContent_GetProperties(eigen)
    if(NOT eigen_POPULATED)
        FetchContent_Populate(eigen)
        set(FUSION_EIGEN_DIR "${eigen_SOURCE_DIR}")
    endif()
endif()

add_library(fusion_core
    src/bayesian/bayesian_network.cpp
    src/fusion/multi_sensor_fusion.cpp
    src/fusion/data_association.cpp
    src/filters/kalman_filter.cpp
    src/confidence/credibility_weighter.cpp
    src/anomaly/anomaly_detector.cpp
    src/sim/scenario_generator.cpp
)
target_include_directories(fusion_core PUBLIC include "${FUSION_EIGEN_DIR}")

if(FUSION_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

if(FUSION_BUILD_BENCHMARKS)
    add_subdirectory(benchmarks)
endif()
