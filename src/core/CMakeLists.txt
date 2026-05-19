add_library(core_lib
    config.cpp
)
target_link_libraries(core_lib
    PUBLIC  Eigen3::Eigen
            spdlog::spdlog
    PRIVATE yaml-cpp
)
target_include_directories(core_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
gs_livo_set_compiler_options(core_lib)
