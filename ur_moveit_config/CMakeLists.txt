cmake_minimum_required(VERSION 3.5)
project(ur_moveit_config)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY config launch rviz srdf
  DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})
ament_python_install_module(${PROJECT_NAME}/launch_common.py)

ament_package()

if(BUILD_TESTING)
  find_package(ur_robot_driver REQUIRED)
  find_package(launch_testing_ament_cmake)

  if(${UR_ROBOT_DRIVER_BUILD_INTEGRATION_TESTS})
    add_launch_test(test/startup_test.py
      TIMEOUT
        180
    )
  endif()
endif()
