add_library(interfaces_lib INTERFACE)
target_include_directories(interfaces_lib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(interfaces_lib INTERFACE core_lib Eigen3::Eigen)
