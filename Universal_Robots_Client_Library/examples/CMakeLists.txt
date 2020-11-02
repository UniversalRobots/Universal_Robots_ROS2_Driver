cmake_minimum_required(VERSION 3.0.2)
project(ur_driver_examples)

#find_package(ur_client_library REQUIRED)

##
## Check C++11 support / enable global pedantic and Wall
##
include(DefineCXX17CompilerFlag)
DEFINE_CXX_17_COMPILER_FLAG(CXX17_FLAG)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic")

add_executable(driver_example
  full_driver.cpp)
target_compile_options(driver_example PUBLIC ${CXX17_FLAG})
target_link_libraries(driver_example ur_client_library::urcl)

add_executable(primary_pipeline_example
  primary_pipeline.cpp)
target_compile_options(primary_pipeline_example PUBLIC ${CXX17_FLAG})
target_link_libraries(primary_pipeline_example ur_client_library::urcl)

add_executable(rtde_client_example
  rtde_client.cpp)
target_compile_options(rtde_client_example PUBLIC ${CXX17_FLAG})
target_link_libraries(rtde_client_example ur_client_library::urcl)
