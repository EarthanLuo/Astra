add_library(sensor_lib INTERFACE)
target_include_directories(sensor_lib INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(sensor_lib INTERFACE core_lib)
