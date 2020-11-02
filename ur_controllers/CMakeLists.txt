cmake_minimum_required(VERSION 3.5)
project(ur_controllers)

add_compile_options(-std=c++11)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(joint_trajectory_controller REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(std_msgs REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_interface
  hardware_interface
  joint_trajectory_controller
  pluginlib
  realtime_tools
  std_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/scaled_joint_trajectory_controller.cpp
  src/speed_scaling_state_controller.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES controller_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
