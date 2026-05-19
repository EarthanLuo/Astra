add_library(lio_lib INTERFACE)
target_include_directories(lio_lib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(lio_lib INTERFACE ekf_lib map_lib)
