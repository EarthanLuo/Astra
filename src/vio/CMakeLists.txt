add_library(vio_lib INTERFACE)
target_include_directories(vio_lib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(vio_lib INTERFACE ekf_lib gs_lib)
