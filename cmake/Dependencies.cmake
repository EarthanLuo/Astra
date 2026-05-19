function(gs_livo_setup_dependencies)
    # --- Eigen3 (required) ---
    find_package(Eigen3 3.4 REQUIRED)
    message(STATUS "Eigen3: ${EIGEN3_VERSION_STRING}")

    # --- spdlog (required) ---
    find_package(spdlog REQUIRED)
    message(STATUS "spdlog: ${spdlog_VERSION}")

    # --- yaml-cpp (required) ---
    find_package(yaml-cpp REQUIRED)
    message(STATUS "yaml-cpp: found")

    # --- OpenCV (Phase 2+, optional) ---
    find_package(OpenCV 4 QUIET COMPONENTS core imgproc highgui)
    if(OpenCV_FOUND)
        message(STATUS "OpenCV: ${OpenCV_VERSION}")
        add_compile_definitions(GS_LIVO_HAS_OPENCV)
    else()
        message(STATUS "OpenCV: not found (will skip camera driver)")
    endif()

    # --- ZeroMQ (Phase 8+, optional) ---
    find_package(cppzmq QUIET)
    if(cppzmq_FOUND)
        message(STATUS "cppzmq: found")
    else()
        message(STATUS "cppzmq: not found (will skip ZMQ visualizer)")
    endif()

    # --- LibTorch (Phase 5+, optional) ---
    if(DEFINED ENV{LibTorch_DIR})
        find_package(Torch QUIET)
        if(Torch_FOUND)
            message(STATUS "LibTorch: found")
        endif()
    endif()

    # --- Catch2 (tests, optional) ---
    if(GS_LIVO_BUILD_TESTS)
        find_package(Catch2 3 QUIET)
        if(Catch2_FOUND)
            message(STATUS "Catch2: found")
        else()
            message(STATUS "Catch2: not found (tests disabled)")
            set(GS_LIVO_BUILD_TESTS OFF CACHE BOOL "Build tests" FORCE)
        endif()
    endif()
endfunction()
