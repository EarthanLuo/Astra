add_library(gs_lib INTERFACE)
target_include_directories(gs_lib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(gs_lib INTERFACE core_lib)
