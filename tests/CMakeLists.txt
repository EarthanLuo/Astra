# GTest is found by the top-level Dependencies.cmake.
# This file is only reached when GS_LIVO_BUILD_TESTS=ON and GTest is available.
# Test targets added as modules are built.
# Each subdirectory follows the pattern:
#   add_executable(<name> <sources>)
#   target_link_libraries(<name> PRIVATE GTest::gtest GTest::gtest_main <module_lib>)
#   gtest_discover_tests(<name>)
