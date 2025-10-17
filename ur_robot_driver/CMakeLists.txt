cmake_minimum_required(VERSION 3.5)
project(ur_robot_driver)

# Default to off as this only works in environments where a new docker container can be started with the appropriate networking, which
# might not be possible e.g. inside the buildfarm
option(
  UR_ROBOT_DRIVER_BUILD_INTEGRATION_TESTS
  "Build integration tests using the start_ursim script"
  OFF
)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall)
  add_compile_options(-Wextra)
  add_compile_options(-Wno-unused-parameter)
endif()
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: You did not request a specific build type: selecting 'RelWithDebInfo'.")
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(ament_cmake REQUIRED)
find_package(backward_ros REQUIRED)
find_package(controller_manager REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ur_client_library REQUIRED)
find_package(ur_dashboard_msgs REQUIRED)
find_package(ur_msgs REQUIRED)

include_directories(include)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_manager
  controller_manager_msgs
  geometry_msgs
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  std_msgs
  std_srvs
  tf2_geometry_msgs
  ur_client_library
  ur_dashboard_msgs
  ur_msgs
)

add_library(ur_robot_driver_plugin
  SHARED
  src/dashboard_client_ros.cpp
  src/hardware_interface.cpp
  src/urcl_log_handler.cpp
  src/robot_state_helper.cpp
)
target_link_libraries(
  ur_robot_driver_plugin
  ur_client_library::urcl
)
target_include_directories(
  ur_robot_driver_plugin
  PRIVATE
  include
)
ament_target_dependencies(
  ur_robot_driver_plugin
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)

#
# dashboard_client
#
add_executable(dashboard_client
  src/dashboard_client_ros.cpp
  src/dashboard_client_node.cpp
  src/urcl_log_handler.cpp
)
target_link_libraries(dashboard_client ur_client_library::urcl)
ament_target_dependencies(dashboard_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${THIS_PACKAGE_INCLUDE_DEPENDS})

#
# ur_ros2_control_node
#
add_executable(ur_ros2_control_node
  src/ur_ros2_control_node.cpp
)
ament_target_dependencies(ur_ros2_control_node
  controller_manager
  rclcpp
)

#
# controller_stopper_node
#
add_executable(controller_stopper_node
  src/controller_stopper.cpp
  src/controller_stopper_node.cpp
)
ament_target_dependencies(controller_stopper_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${THIS_PACKAGE_INCLUDE_DEPENDS})

#
# robot_state_helper
#
add_executable(robot_state_helper
  src/robot_state_helper.cpp
  src/robot_state_helper_node.cpp
  src/urcl_log_handler.cpp
)
target_link_libraries(robot_state_helper ur_client_library::urcl)
ament_target_dependencies(robot_state_helper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(urscript_interface
  src/urscript_interface.cpp
)
ament_target_dependencies(urscript_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${THIS_PACKAGE_INCLUDE_DEPENDS})

#
# trajectory_until_node
#
add_executable(trajectory_until_node
  src/trajectory_until_node.cpp
)
target_link_libraries(trajectory_until_node PUBLIC
  rclcpp::rclcpp
  ${std_msgs_TARGETS}
  rclcpp_action::rclcpp_action
  ${ur_msgs_TARGETS}
)

install(
  TARGETS
    controller_stopper_node
    dashboard_client
    robot_state_helper
    trajectory_until_node
    ur_ros2_control_node
    urscript_interface
  DESTINATION lib/${PROJECT_NAME}
)

# INSTALL
install(
  TARGETS ur_robot_driver_plugin
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ur_robot_driver_plugin
)

install(DIRECTORY resources
  DESTINATION share/${PROJECT_NAME}
  REGEX "/ursim/" EXCLUDE
)


install(DIRECTORY include/
  DESTINATION include
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Install Python execs
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/tool_communication.py
  scripts/example_move.py
  scripts/start_ursim.sh
  examples/examples.py
  examples/force_mode.py
  examples/move_until_example.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS scripts/wait_dashboard_server.sh
  DESTINATION bin
)

install(PROGRAMS scripts/start_ursim.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

if(BUILD_TESTING)
  find_package(ur_controllers REQUIRED)
  find_package(ur_description REQUIRED)
  find_package(ur_msgs REQUIRED)
  find_package(launch_testing_ament_cmake)

  add_launch_test(test/test_mock_hardware.py
    TIMEOUT
      800
  )

  if(${UR_ROBOT_DRIVER_BUILD_INTEGRATION_TESTS})
    add_launch_test(test/launch_args.py
      TIMEOUT
        180
    )
    add_launch_test(test/dashboard_client.py
      TIMEOUT
        180
    )
    add_launch_test(test/integration_test_scaled_joint_controller.py
      TIMEOUT
        800
    )
    add_launch_test(test/integration_test_controller_switch.py
      TIMEOUT
        800
    )
    add_launch_test(test/integration_test_force_mode.py
      TIMEOUT
        800
    )
    add_launch_test(test/urscript_interface.py
      TIMEOUT
        500
    )
    add_launch_test(test/integration_test_config_controller.py
      TIMEOUT
        800
    )
    add_launch_test(test/integration_test_passthrough_controller.py
      TIMEOUT
        800
    )
    add_launch_test(test/integration_test_io_controller.py
      TIMEOUT
        800
    )
    add_launch_test(test/integration_test_tool_contact.py
      TIMEOUT
        800
    )
    add_launch_test(test/integration_test_trajectory_until.py
      TIMEOUT
        800
    )
  endif()
endif()
