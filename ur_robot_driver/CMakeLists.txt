cmake_minimum_required(VERSION 3.5)
project(ur_robot_driver)

add_compile_options(-std=c++11)
add_compile_options(-Wall)
add_compile_options(-Wextra)
add_compile_options(-Wno-unused-parameter)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: You did not request a specific build type: selecting 'RelWithDebInfo'.")
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

find_package(ament_cmake REQUIRED)
find_package(ur_client_library REQUIRED)
find_package(ur_dashboard_msgs REQUIRED)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ur_dashboard_msgs
)

add_library(ur_robot_driver_plugin
  src/dashboard_client_ros.cpp
  src/hardware_interface.cpp
)
target_link_libraries(ur_robot_driver_plugin ur_client_library::urcl)
ament_target_dependencies(ur_robot_driver_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(ur_robot_driver_node
  src/dashboard_client_ros.cpp
  src/hardware_interface.cpp
  src/hardware_interface_node.cpp
)
target_link_libraries(ur_robot_driver_node ${catkin_LIBRARIES} ur_client_library::urcl)
ament_target_dependencies(ur_robot_driver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(dashboard_client
  src/dashboard_client_ros.cpp
  src/dashboard_client_node.cpp
)
target_link_libraries(dashboard_client ${catkin_LIBRARIES} ur_client_library::urcl)
ament_target_dependencies(dashboard_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(robot_state_helper
  src/robot_state_helper.cpp
  src/robot_state_helper_node.cpp
)
target_link_libraries(robot_state_helper ${catkin_LIBRARIES} ur_client_library::urcl)
ament_target_dependencies(robot_state_helper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${THIS_PACKAGE_INCLUDE_DEPENDS})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest(test/driver.test)
endif()


install(TARGETS ur_robot_driver_plugin ur_robot_driver_node robot_state_helper dashboard_client
  ARCHIVE DESTINATION lib}
  LIBRARY DESTINATION lib}
  RUNTIME DESTINATION bin}
)

install(PROGRAMS scripts/tool_communication
  DESTINATION bin
)

install(DIRECTORY config launch resources
  DESTINATION share/${PROJECT_NAME}
)


install(DIRECTORY include/
  DESTINATION include
)

install(FILES hardware_interface_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_package()
