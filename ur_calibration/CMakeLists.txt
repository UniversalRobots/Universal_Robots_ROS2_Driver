cmake_minimum_required(VERSION 3.5)
project(ur_calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-Wno-unused-parameter)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: You did not request a specific build type: selecting 'RelWithDebInfo'.")
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ur_robot_driver REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ur_client_library REQUIRED)

set(YAML_CPP_INCLUDE_DIRS ${YAML_CPP_INCLUDE_DIR})

###########
## Build ##
###########

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

add_executable(calibration_correction
  src/calibration.cpp
  src/calibration_consumer.cpp
  src/calibration_correction.cpp
)
target_include_directories(calibration_correction PRIVATE include)
target_link_libraries(calibration_correction
        ${YAML_CPP_LIBRARIES}
        ur_client_library::urcl
)
ament_target_dependencies(
        calibration_correction
        rclcpp
        ur_robot_driver
)

install(TARGETS calibration_correction
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
        DESTINATION include
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)

## Add gtest based cpp test target and link libraries
if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)

  # Get the first item (it will be the build space version of the build path).
  list(GET ament_index_build_path 0 ament_index_build_path)
  if(WIN32)
    # On Windows prevent CMake errors and prevent it being evaluated as a list.
    string(REPLACE "\\" "/" ament_index_build_path "${ament_index_build_path}")
  endif()

  ament_add_gmock(
          calibration_test
          test/calibration_test.cpp
          src/calibration.cpp
  )
  target_include_directories(calibration_test PRIVATE include ${EIGEN3_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS})
  target_link_libraries(calibration_test ${rclcpp_LIBRARIES} ${EIGEN3_LIBRARIES} ${YAML_CPP_LIBRARIES} ur_client_library::urcl)
endif()

ament_export_include_directories(
        include
)
ament_export_dependencies(
        ur_robot_driver
        rclcpp
)
ament_package()
