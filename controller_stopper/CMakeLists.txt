cmake_minimum_required(VERSION 3.5)
project(controller_stopper)

add_compile_options(-std=c++11)
add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Wno-unused-parameter)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: You did not request a specific build type: selecting 'RelWithDebInfo'.")
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(controller_manager_msgs REQUIRED)

include_directories(include)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  std_srvs
  controller_manager_msgs
)


add_executable(controller_stopper_node
  src/controller_stopper.cpp
  src/controller_stopper_node.cpp
)
target_include_directories(controller_stopper_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(controller_stopper_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(TARGETS controller_stopper_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
