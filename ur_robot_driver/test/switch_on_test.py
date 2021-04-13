#!/usr/bin/env python
# Copyright 2019, FZI Forschungszentrum Informatik
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import sys
import unittest

import actionlib
import rospy

from ur_dashboard_msgs.msg import RobotMode, SetModeAction, SetModeGoal

PKG = "ur_robot_driver"
NAME = "switch_on_test"


class IOTest(unittest.TestCase):
    def __init__(self, *args):
        super(IOTest, self).__init__(*args)

        rospy.init_node("switch_on_robot")
        self.client = actionlib.SimpleActionClient("/ur_hardware_interface/set_mode", SetModeAction)
        timeout = rospy.Duration(30)
        try:
            self.client.wait_for_server(timeout)
        except rospy.exceptions.ROSException as err:
            self.fail(
                "Could not reach set_mode action. Make sure that the driver is actually running."
                " Msg: {}".format(err)
            )

    def test_switch_on(self):
        """Test to set an IO and check whether it has been set."""
        goal = SetModeGoal()
        goal.target_robot_mode = RobotMode.RUNNING
        goal.play_program = False  # we use headless mode during tests

        self.client.send_goal(goal)
        self.client.wait_for_result()

        self.assertTrue(self.client.get_result().success)


if __name__ == "__main__":
    import rostest

    rostest.run(PKG, NAME, IOTest, sys.argv)
