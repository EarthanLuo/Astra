# Catch2 is found by the top-level Dependencies.cmake.
# This file is only reached when GS_LIVO_BUILD_TESTS=ON and Catch2 is available.
# Test targets added as modules are built.
# Each subdirectory follows the pattern:
#   add_executable(<name> <sources>)
#   target_link_libraries(<name> PRIVATE Catch2::Catch2 <module_lib>)
#   catch_discover_tests(<name>)
