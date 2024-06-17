
ur_calibration
==============

Package for extracting the factory calibration from a UR robot and changing it to be used by ``ur_description`` to gain a correct URDF model.

Each UR robot is calibrated inside the factory giving exact forward and inverse kinematics. To also
make use of this in ROS, you first have to extract the calibration information from the robot.

Though this step is not necessary, to control the robot using this driver, it is highly recommended
to do so, as end effector positions might be off in the magnitude of centimeters.

.. toctree::
   :maxdepth: 2

   usage
   algorithm
