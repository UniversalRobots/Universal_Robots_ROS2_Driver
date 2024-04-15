Installation of the ur_robot_driver
===================================

You can either install this driver from binary packages as shown above or build it from source. We
recommend a binary package installation unless you want to join development and submit changes.

Install from binary packages
----------------------------

1. `Install ROS2 <https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html>`_. This
   branch supports only ROS2 Rolling. For other ROS2 versions, please see the respective branches.
2. Install the driver using

   .. code-block:: bash

     sudo apt-get install ros-${ROS_DISTRO}-ur


Build from source
-----------------

Before building from source please make sure that you actually need to do that. Building from source
might require some special treatment, especially when it comes to dependency management.
Dependencies might change from time to time. Upstream packages (such as the library) might change
their features / API which require changes in this repo. Therefore, this repo's source builds might
require upstream repositories to be present in a certain version as otherwise builds might fail.
Starting from scratch following exactly the steps below should always work, but simply pulling and
building might fail occasionally.

1. `Install ROS2 <https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html>`_. This
   branch supports only ROS2 Rolling. For other ROS2 versions, please see the respective branches.

   Once installed, please make sure to actually `source ROS2 <https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#source-the-setup-files>`_ before proceeding.

3. Make sure that ``colcon``, its extensions and ``vcs`` are installed:

   .. code-block:: bash

     sudo apt install python3-colcon-common-extensions python3-vcstool


4. Create a new ROS2 workspace:

   .. code-block:: bash

     export COLCON_WS=~/workspace/ros_ur_driver
     mkdir -p $COLCON_WS/src

5. Clone relevant packages (replace ``<branch>`` with ``humble``, ``iron`` or ``main`` for rolling), install dependencies, compile, and source the workspace by using:

   .. code-block:: bash

     cd $COLCON_WS
     git clone -b <branch> https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git src/Universal_Robots_ROS2_Driver
     vcs import src --skip-existing --input src/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver-not-released.${ROS_DISTRO}.repos
     rosdep update
     rosdep install --ignore-src --from-paths src -y
     colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
     source install/setup.bash

6. When consecutive pulls lead to build errors it is possible that you'll have to build an upstream
   package from source, as well. See the [detailed build status](ci_status.md). When the binary builds are red, but
   the semi-binary builds are green, you need to build the upstream dependencies from source. The
   easiest way to achieve this, is using the repos file:

   .. code-block:: bash

     cd $COLCON_WS
     vcs import src --skip-existing --input src/Universal_Robots_ROS2_Driver/Universal_Robots_ROS2_Driver.${ROS_DISTRO}.repos
     rosdep update
     rosdep install --ignore-src --from-paths src -y
