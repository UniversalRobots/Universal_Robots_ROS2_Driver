cmake_minimum_required(VERSION 3.5)
project(ur_calibration)

add_compile_options(-std=c++11)

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

find_package(Boost REQUIRED COMPONENTS filesystem)

set(YAML_CPP_INCLUDE_DIRS ${YAML_CPP_INCLUDE_DIR})

###########
## Build ##
###########

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(calibration_correction
  src/calibration.cpp
  src/calibration_consumer.cpp
  src/calibration_correction.cpp
)
target_include_directories(calibration_correction PRIVATE include ${Boost_INCLUDE_DIRS})
target_link_libraries(calibration_correction
        ${Boost_LIBRARIES}
        ${YAML_CPP_LIBRARIES}
        ur_client_library::urcl
)
ament_target_dependencies(
        calibration_correction
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
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

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if (BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
        include
)
ament_export_dependencies(
        ur_robot_driver
        rclcpp
)
ament_package()
