add_subdirectory(core)
add_subdirectory(interfaces)
add_subdirectory(ekf)
add_subdirectory(sensor)
add_subdirectory(imu)
add_subdirectory(lio)
add_subdirectory(map)
add_subdirectory(gs)
add_subdirectory(vio)
add_subdirectory(orchestrator)

add_executable(gs_livo main.cpp)
target_link_libraries(gs_livo PRIVATE
    core_lib
    orchestrator_lib
)

gs_livo_set_compiler_options(gs_livo)
