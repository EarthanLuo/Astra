add_library(map_lib INTERFACE)
target_include_directories(map_lib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(map_lib INTERFACE core_lib)
