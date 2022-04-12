// Copyright 2019, FZI Forschungszentrum Informatik, Created on behalf of Universal Robots A/S
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner exner@fzi.de
 * \date    2019-06-12
 *
 * \author  Mads Holm Peters
 * \date    2022-02-25
 *
 */
//----------------------------------------------------------------------

#include "rclcpp/rclcpp.hpp"
#include "ur_robot_driver/controller_stopper.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("controller_stopper_node");

  bool headless_mode = node->declare_parameter<bool>("headless_mode", false);
  node->get_parameter<bool>("headless_mode", headless_mode);
  bool joint_controller_active = node->declare_parameter<bool>("joint_controller_active", true);
  node->get_parameter<bool>("joint_controller_active", joint_controller_active);

  // If headless mode is not active, but the joint controllers are we should stop the joint controllers during startup
  // of the node
  bool stop_controllers_on_startup = false;
  if (joint_controller_active == true && headless_mode == false) {
    stop_controllers_on_startup = true;
  }

  ControllerStopper stopper(node, stop_controllers_on_startup);

  rclcpp::spin(node);

  return 0;
}
