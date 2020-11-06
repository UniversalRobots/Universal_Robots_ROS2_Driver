# Universal Robots ROS2 Driver

Alpha version of the ROS2 Universal Robots driver. Should be transferred to the Universal Robots org when ready.

## Build Instructions

To build this package in a ROS Foxy workspace, clone this repo
into the `src/` directory, then:

```
# Clone source-based dependencies into src/ directory
vcs import --skip-existing --input src/Universal_Robots_ROS2_Driver/.repos.yaml src

# Install package-based dependencies
rosdep install -y --rosdistro $ROS_DISTRO --ignore-src --from-paths src

# Build sources
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```
