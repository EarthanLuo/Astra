cmake_minimum_required(VERSION 3.24)
project(gs-livo VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# CUDA is optional — only enable when toolkit is installed (Phase 5+)
find_package(CUDAToolkit QUIET)
if(CUDAToolkit_FOUND)
    enable_language(CUDA)
    set(CMAKE_CUDA_STANDARD 20)
    set(CMAKE_CUDA_STANDARD_REQUIRED ON)
    message(STATUS "CUDA found: ${CUDAToolkit_VERSION}")
else()
    message(STATUS "CUDA not found — CUDA targets will be skipped")
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Build type" FORCE)
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
include(CompilerOptions)
include(CudaOptions)
include(Dependencies)
include(Sanitizers)

option(GS_LIVO_BUILD_TESTS "Build tests" ON)
option(GS_LIVO_BUILD_VIEWER "Build viewer" OFF)
option(GS_LIVO_USE_SANITIZER "Enable sanitizers (debug builds)" OFF)

gs_livo_setup_dependencies()

add_subdirectory(src)

if(GS_LIVO_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

include(InstallRules)
