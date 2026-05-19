add_library(imu_lib INTERFACE)
target_include_directories(imu_lib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(imu_lib INTERFACE ekf_lib)
