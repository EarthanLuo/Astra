add_library(ekf_lib INTERFACE)
target_include_directories(ekf_lib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ekf_lib INTERFACE core_lib Eigen3::Eigen)
