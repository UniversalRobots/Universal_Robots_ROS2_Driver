cmake_minimum_required(VERSION 3.5)
project(ur_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

# Sets an environment variable called IGN_GAZEBO_RESOURCE_PATH via environment hooks
# This is so that Ignition Gazebo knows the location of meshes
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in")

ament_package()
