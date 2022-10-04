# ur_calibration

Package for extracting the factory calibration from a UR robot and changing it to be used by `ur_description` to gain a correct URDF model.

Each UR robot is calibrated inside the factory giving exact forward and inverse kinematics. To also
make use of this in ROS, you first have to extract the calibration information from the robot.

Though this step is not necessary, to control the robot using this driver, it is highly recommended
to do so, as end effector positions might be off in the magnitude of centimeters.

## Nodes
### calibration_correction
This node extracts calibration information directly from a robot, calculates the URDF correction and
saves it into a .yaml file.

In the launch folder of the ur_calibration package is a helper script:

```bash
$ ros2 launch ur_calibration calibration_correction.launch.py \
robot_ip:=<robot_ip> target_filename:="${HOME}/my_robot_calibration.yaml"
```

For the parameter `robot_ip` insert the IP address on which the ROS pc can reach the robot. As
`target_filename` provide an absolute path where the result will be saved to.

## Creating a calibration / launch package for all local robots
When dealing with multiple robots in one organization it might make sense to store calibration data
into a package dedicated to that purpose only. To do so, create a new package (if it doesn't already
exist)

```bash
# Replace your actual colcon_ws folder
$ cd <colcon_ws>/src
$ ros2 pkg  create <organization_name>_ur_launch --build-type ament_cmake  --dependencies ur_client_library \
--description "Package containing calibrations and launch files for our UR robots."
# Create a skeleton package
$ mkdir -p <organization_name>_ur_launch/etc
$ mkdir -p <organization_name>_ur_launch/launch
$ echo 'install(DIRECTORY etc launch DESTINATION share/${PROJECT_NAME})' >> <organization_name>_ur_launch/CMakeLists.txt
$ colcon build --packages-select <organization_name>_ur_launch
```

We can use the new package to store the calibration data in that package. We recommend naming each
robot individually, e.g. *ex-ur10-1*.

```bash
$ ros2 launch ur_calibration calibration_correction.launch.py \
robot_ip:=<robot_ip> \
target_filename:="$(ros2 pkg prefix <organization_name>_ur_launch)/share/<organization_name>_ur_launch/etc/ex-ur10-1_calibration.yaml"
```

To make life easier, we create a launchfile for this particular robot. We base it upon the
respective launchfile in the driver:

```bash
# Replace your actual colcon_ws folder
$ cd <colcon_ws>/src/<organization_name>_ur_launch/launch
$ cp $(ros2 pkg prefix ur_robot_driver)/share/ur_robot_driver/launch/ur_control.launch.py ex-ur10-1.launch.py
```

Next, modify the parameter section of the new launchfile to match your actual calibration:

```py
kinematics_params = PathJoinSubstitution(
        [FindPackageShare("<organization_name>_ur_launch"), "etc", "", "ex-ur10-1_calibration.yaml"]
    )

```

Then, anybody cloning this repository can startup the robot simply by launching

```bash
# Replace your actual colcon_ws folder
$ cd <colcon_ws>
$ colcon build --packages-select <organization_name>_ur_launch
$ ros2 launch <organization_name>_ur_launch ex-ur10-1.launch.py
robot_ip:=xxx.yyy.zzz.www ur_type:=ur5e  use_fake_hardware:=false launch_rviz:=true
```
