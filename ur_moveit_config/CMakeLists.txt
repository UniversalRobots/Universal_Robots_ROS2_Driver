cmake_minimum_required(VERSION 3.5)
project(ur_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config rviz srdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
