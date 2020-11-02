cmake_minimum_required(VERSION 3.0.2)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../CMakeModules/" ${CMAKE_MODULE_PATH})

project(ur_client_library_tests)

# This will allow cmake 3.0 and above to fail on missing link targets (when specified with an double colon syntax)
# Note that not all packages are support the colon syntax, but if they do it is recomended to use this syntax, since it is less error prone.
if(POLICY CMP0028)
  cmake_policy(SET CMP0028 NEW)
endif()

if(POLICY CMP0054)
  cmake_policy(SET CMP0054 NEW)
endif()

# Find dependencies for testing
include(FindGTestPackage)
find_gtest_package(GTEST_LIBRARIES)
if (NOT TARGET ur_client_library::urcl)
  find_package(ur_client_library REQUIRED)
endif()

# Check C++11 support
include(DefineCXX17CompilerFlag)
DEFINE_CXX_17_COMPILER_FLAG(CXX17_FLAG)


# Build Tests
if (INTEGRATION_TESTS)
  # Integration tests require a robot reachable at 192.168.56.101. Therefore, they have to be
  # activated separately.
  add_executable(rtde_tests test_rtde_client.cpp)
  target_compile_options(rtde_tests PRIVATE ${CXX17_FLAG})
  target_include_directories(rtde_tests PRIVATE ${GTEST_INCLUDE_DIRS})
  target_link_libraries(rtde_tests PRIVATE ur_client_library::urcl ${GTEST_LIBRARIES})
  gtest_add_tests(TARGET      rtde_tests
                  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
else()
  message(STATUS "Skipping integration tests.")
endif()

add_executable(primary_parser_tests test_primary_parser.cpp)
target_compile_options(primary_parser_tests PRIVATE ${CXX17_FLAG})
target_include_directories(primary_parser_tests PRIVATE ${GTEST_INCLUDE_DIRS})
target_link_libraries(primary_parser_tests PRIVATE ur_client_library::urcl ${GTEST_LIBRARIES})
gtest_add_tests(TARGET      primary_parser_tests
)

add_executable(rtde_data_package test_rtde_data_package.cpp)
target_compile_options(rtde_data_package PRIVATE ${CXX17_FLAG})
target_include_directories(rtde_data_package PRIVATE ${GTEST_INCLUDE_DIRS})
target_link_libraries(rtde_data_package PRIVATE ur_client_library::urcl ${GTEST_LIBRARIES})
gtest_add_tests(TARGET      rtde_data_package
)

add_executable(rtde_parser_tests test_rtde_parser.cpp)
target_compile_options(rtde_parser_tests PRIVATE ${CXX17_FLAG})
target_include_directories(rtde_parser_tests PRIVATE ${GTEST_INCLUDE_DIRS})
target_link_libraries(rtde_parser_tests PRIVATE ur_client_library::urcl ${GTEST_LIBRARIES})
gtest_add_tests(TARGET      rtde_parser_tests
)
