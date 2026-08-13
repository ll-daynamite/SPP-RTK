cmake_minimum_required(VERSION 3.20)

project(SPP_RTK LANGUAGES CXX)

if(NOT WIN32)
    message(FATAL_ERROR "The realtime socket implementation currently requires Windows.")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_path(
    EIGEN3_INCLUDE_DIR
    NAMES Eigen/Dense
    HINTS "${EIGEN3_ROOT}" "$ENV{EIGEN3_ROOT}"
    PATH_SUFFIXES eigen3)

if(NOT EIGEN3_INCLUDE_DIR)
    message(FATAL_ERROR
        "Eigen was not found. Set EIGEN3_INCLUDE_DIR to the directory that contains Eigen/Dense.")
endif()

add_executable(
    SPP-RTK
    Application.cpp
    ApplicationConfig.cpp
    CoordinateTransformation.cpp
    Decode.cpp
    ErrorCorrection.cpp
    GNSS_PPP.cpp
    KF.cpp
    lambda.cpp
    Position.cpp
    RTK.cpp
    RtkWorkflow.cpp
    sockets.cpp
    SPP_SPV.cpp
    SppWorkflow.cpp
    TimeConversion.cpp
    WorkflowSupport.cpp)

target_include_directories(SPP-RTK PRIVATE "${EIGEN3_INCLUDE_DIR}")
target_compile_definitions(SPP-RTK PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
target_link_libraries(SPP-RTK PRIVATE ws2_32)

if(MSVC)
    target_compile_options(SPP-RTK PRIVATE /W3 /wd4828)

    # The original course sources are GBK. New orchestration files are UTF-8.
    # Keeping the source-specific settings avoids changing the legacy algorithm
    # files solely for an encoding conversion.
    set_source_files_properties(
        Application.cpp
        ApplicationConfig.cpp
        GNSS_PPP.cpp
        RtkWorkflow.cpp
        SppWorkflow.cpp
        WorkflowSupport.cpp
        PROPERTIES COMPILE_OPTIONS /utf-8)
endif()
