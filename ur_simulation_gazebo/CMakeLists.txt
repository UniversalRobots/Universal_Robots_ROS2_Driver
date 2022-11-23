cmake_minimum_required(VERSION 3.5)
project(ur_simulation_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch test
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
