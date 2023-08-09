.. _ursim_docker:

Setup URSim with Docker
=======================
URSim is the offline simulator by Universal Robots. Packed into a remote or virtual machine it acts almost
identically to a real robot connected over the network. While it is possible to get URSim running
locally on a Linux system or inside a VirtualBox virtual machine, we will focus on getting things
setup using Docker. Using Docker for your simulated robot allows you to very quickly spin up a robot
testing instance with very little computational overload.

This guide will assume that you have Docker already installed and setup such that you can startup
Docker containers using your current user.

Start a URSim docker container
------------------------------

To startup a simulated robot run the following command. This will start a Docker container named
``ursim`` and startup a simulated UR5e robot. It exposes ports 5900 and 6080 for the browser-based
polyscope access. Note that this will expose the simulated robot to your local area network if you
don't have any further level of security such as a firewall active. To prevent this, you can either
skip the port forwarding instructions (skip the two ``-p port:port`` statements) in which case
you'll have to use the container's IP address to access the polyscope gui rather than ``localhost`` or
you can restrict the port forwarding to a certain network interface (such as the looppack interface)
see Docker's upstream documentation on port exposure for further information.

.. code-block:: bash

   docker run --rm -it -p 5900:5900 -p 6080:6080 --name ursim universalrobots/ursim_e-series

External Control
----------------

To use the external control functionality, we will need the ``external_control`` URCap installed on
the robot and a program containing its *ExternalControl* program node. Both can be prepared on the
host machine either by creating an own Dockerfile containing those or by mounting two folders
containing installed URCaps and programs. See the Dockerfile's upstream `documentation <https://hub.docker.com/r/universalrobots/ursim_e-series>`_.

In this example, we will bind-mount a folder for the programs and URCaps. First, let's create a
local folder where we can store things inside:

.. code-block:: bash

   mkdir -p ${HOME}/.ursim/programs
   mkdir -p ${HOME}/.ursim/urcaps

Then, we can "install" the URCap by placing its ``.jar`` file inside the urcaps folder

.. code-block:: bash

   URCAP_VERSION=1.0.5 # latest version as if writing this
   curl -L -o ${HOME}/.ursim/urcaps/externalcontrol-${URCAP_VERSION}.jar \
     https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases/download/v${URCAP_VERSION}/externalcontrol-${URCAP_VERSION}.jar

With this, start your URSim containers with the following command:

.. code-block:: bash

   docker run --rm -it -p 5900:5900 -p 6080:6080 -v ${HOME}/.ursim/urcaps:/urcaps -v ${HOME}/.ursim/programs:/ursim/programs --name ursim universalrobots/ursim_e-series

With this, you should be able to setup the ``external_control`` URCap and create a program as
described in :ref:`URCap setup guide <install-urcap-e-series>`.

Network setup
-------------

As described above, you can always start the URSim container using the default network setup. As long
as you don't have any other docker containers running, it will most probably always get the same IP
address assigned every time. However, to make things a bit more explicit, we can setup our own
docker network where we can assign a static IP address to our URSim container.

.. code-block:: bash

   docker network create --subnet=192.168.56.0/24 ursim_net
   docker run --rm -it -p 5900:5900 -p 6080:6080 --net ursim_net --ip 192.168.56.101 universalrobots/ursim_e-series

The above commands first create a network for docker and then create a container with the URSim
image attaching to this network.

As we now have a fixed IP address we can also skip the port exposure as we know the robot's IP
address. The VNC web server will be available at `<http://192.168.56.101:6080/vnc.html>`_

Script startup
--------------

All of the above is put together in a script in the ``ur_client_library`` package.

.. code-block:: bash

   ros2 run ur_client_library start_ursim.sh

This will start a URSim docker container running on ``192.168.56.101`` with the ``external_control``
URCap preinstalled. Created programs and installation changes will be stored persistently inside
``${HOME}/.ursim/programs``.

With this, you can run

.. code-block:: bash

   ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.56.101
