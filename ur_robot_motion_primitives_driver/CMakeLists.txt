cmake_minimum_required(VERSION 3.8)
project(ur_robot_motion_primitives_driver)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ament_cmake_gmock REQUIRED)
find_package(ros2_control_test_assets REQUIRED)
find_package(motion_primitives_forward_controller REQUIRED)
find_package(rclpy REQUIRED)
find_package(industrial_robot_motion_interfaces REQUIRED)
# find_package(urcl REQUIRED)
find_package(ur_client_library REQUIRED)
find_package(ur_robot_driver REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  ${ur_robot_driver_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${motion_primitives_forward_controller_INCLUDE_DIRS}
  )

add_library(
  ur_robot_motion_primitives_driver
  SHARED
  src/motion_primitives_ur_driver.cpp
)
target_include_directories(
  ur_robot_motion_primitives_driver
  PUBLIC
  include
)
ament_target_dependencies(
  ur_robot_motion_primitives_driver
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  motion_primitives_forward_controller
  ur_client_library
  tf2
  tf2_geometry_msgs
)
# prevent pluginlib from using boost
target_compile_definitions(ur_robot_motion_primitives_driver PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface ur_robot_motion_primitives_driver.xml)

install(
  TARGETS
  ur_robot_motion_primitives_driver
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install Python execs
ament_python_install_package(${PROJECT_NAME})

# Install the Python script as an executable
install(PROGRAMS
  scripts/send_single_dummy_motion_primitive.py  # Python script
  scripts/send_multiple_dummy_motion_primitives_after_checking_status.py 
  scripts/send_multiple_dummy_motion_primitives.py 
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)

  ament_add_gmock(test_motion_primitives_ur_driver test/test_motion_primitives_ur_driver.cpp)
  target_include_directories(test_motion_primitives_ur_driver PRIVATE include)
  ament_target_dependencies(
    test_motion_primitives_ur_driver
    hardware_interface
    pluginlib
    ros2_control_test_assets
  )
endif()

ament_export_include_directories(
  include
)
ament_export_libraries(
  ur_robot_motion_primitives_driver
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
